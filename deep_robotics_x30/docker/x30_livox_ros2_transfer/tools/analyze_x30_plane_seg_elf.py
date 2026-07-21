#!/usr/bin/env python3
"""Extract annotated AArch64 functions from the factory X30 plane_seg ELF."""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path
from typing import Iterable

try:
    from capstone import CS_ARCH_ARM64, CS_MODE_ARM, CS_OP_IMM, Cs
    from capstone.arm64 import ARM64_OP_MEM, ARM64_OP_REG
    from elftools.elf.elffile import ELFFile
    from elftools.elf.relocation import RelocationSection
except ImportError as exc:  # pragma: no cover - exercised by operators.
    raise SystemExit(
        "Install tools/requirements-plane-seg-analysis.txt before running this tool."
    ) from exc


def compact_symbol_name(name: str) -> str:
    """Recover the readable nested-name prefix from common Itanium symbols."""
    if not name.startswith("_ZN"):
        return name
    cursor = 3
    parts: list[str] = []
    while cursor < len(name) and name[cursor] != "E":
        start = cursor
        while cursor < len(name) and name[cursor].isdigit():
            cursor += 1
        if start == cursor:
            break
        length = int(name[start:cursor])
        part = name[cursor : cursor + length]
        if len(part) != length:
            break
        parts.append(part)
        cursor += length
    return "::".join(parts) if parts else name


def build_symbol_map(elf: ELFFile) -> tuple[dict[int, str], list[tuple[int, int, str]]]:
    names: dict[int, str] = {}
    functions: dict[tuple[int, int, str], None] = {}
    for section_name in (".symtab", ".dynsym"):
        section = elf.get_section_by_name(section_name)
        if section is None:
            continue
        for symbol in section.iter_symbols():
            address = int(symbol["st_value"])
            size = int(symbol["st_size"])
            name = symbol.name
            if address and name:
                names.setdefault(address, name)
            if (
                address
                and size
                and name
                and symbol["st_info"]["type"] == "STT_FUNC"
            ):
                functions[(address, size, name)] = None

    plt = elf.get_section_by_name(".plt")
    dynsym = elf.get_section_by_name(".dynsym")
    rela_plt = elf.get_section_by_name(".rela.plt")
    if (
        plt is not None
        and dynsym is not None
        and isinstance(rela_plt, RelocationSection)
    ):
        # AArch64 uses a 32-byte PLT header followed by 16-byte entries.
        for index, relocation in enumerate(rela_plt.iter_relocations()):
            symbol = dynsym.get_symbol(relocation["r_info_sym"])
            names[int(plt["sh_addr"]) + 32 + index * 16] = symbol.name + "@plt"

    return names, sorted(functions)


def find_functions(
    functions: Iterable[tuple[int, int, str]], selectors: list[str]
) -> list[tuple[int, int, str]]:
    selected: list[tuple[int, int, str]] = []
    seen: set[int] = set()
    for address, size, name in functions:
        readable = compact_symbol_name(name)
        if any(selector in name or selector in readable for selector in selectors):
            if address not in seen:
                selected.append((address, size, name))
                seen.add(address)
    return selected


def read_virtual(elf: ELFFile, address: int, size: int) -> bytes | None:
    for section in elf.iter_sections():
        start = int(section["sh_addr"])
        length = int(section["sh_size"])
        if start <= address and address + size <= start + length:
            offset = address - start
            return section.data()[offset : offset + size]
    return None


def printable_string(elf: ELFFile, address: int, limit: int = 160) -> str | None:
    data = read_virtual(elf, address, limit)
    if not data:
        return None
    value = data.split(b"\0", 1)[0]
    if len(value) < 4 or any(byte < 0x20 or byte > 0x7E for byte in value):
        return None
    return value.decode("ascii")


def constant_annotation(elf: ELFFile, address: int, register: str) -> str | None:
    text = printable_string(elf, address)
    if text is not None:
        return f"string={text!r}"

    size = 8 if register.startswith(("x", "d")) else 4
    data = read_virtual(elf, address, size)
    if data is None:
        return None
    if register.startswith("s"):
        value = struct.unpack("<f", data)[0]
        if math.isfinite(value):
            return f"float32={value:.9g}"
    if register.startswith("d"):
        value = struct.unpack("<d", data)[0]
        if math.isfinite(value):
            return f"float64={value:.12g}"
    return f"raw={data.hex()}"


def disassemble_function(
    elf: ELFFile,
    symbols: dict[int, str],
    function: tuple[int, int, str],
) -> list[str]:
    address, size, name = function
    code = read_virtual(elf, address, size)
    if code is None:
        raise ValueError(f"cannot map function bytes at 0x{address:x}")

    disassembler = Cs(CS_ARCH_ARM64, CS_MODE_ARM)
    disassembler.detail = True
    known_addresses: dict[int, int] = {}
    output = [
        f"FUNCTION 0x{address:08x} size={size} {compact_symbol_name(name)}",
        f"MANGLED {name}",
    ]

    for instruction in disassembler.disasm(code, address):
        comments: list[str] = []
        operands = instruction.operands

        if instruction.mnemonic in ("adr", "adrp") and len(operands) >= 2:
            if operands[0].type == ARM64_OP_REG and operands[1].type == CS_OP_IMM:
                destination = operands[0].reg
                known_addresses[destination] = int(operands[1].imm)
        elif instruction.mnemonic == "add" and len(operands) >= 3:
            if (
                operands[0].type == ARM64_OP_REG
                and operands[1].type == ARM64_OP_REG
                and operands[2].type == CS_OP_IMM
                and operands[1].reg in known_addresses
            ):
                destination = operands[0].reg
                known_addresses[destination] = (
                    known_addresses[operands[1].reg] + int(operands[2].imm)
                )
                text = printable_string(elf, known_addresses[destination])
                if text is not None:
                    comments.append(f"string={text!r}")
        elif instruction.mnemonic == "mov" and len(operands) >= 2:
            if operands[0].type == ARM64_OP_REG and operands[1].type == ARM64_OP_REG:
                if operands[1].reg in known_addresses:
                    known_addresses[operands[0].reg] = known_addresses[operands[1].reg]
        elif instruction.mnemonic.startswith("ldr") and len(operands) >= 2:
            if operands[0].type == ARM64_OP_REG and operands[1].type == ARM64_OP_MEM:
                memory = operands[1].mem
                if memory.base in known_addresses and memory.index == 0:
                    constant_address = known_addresses[memory.base] + int(memory.disp)
                    register = instruction.reg_name(operands[0].reg)
                    annotation = constant_annotation(elf, constant_address, register)
                    if annotation is not None:
                        comments.append(f"0x{constant_address:x} {annotation}")

        if instruction.mnemonic in ("bl", "b") and operands:
            if operands[0].type == CS_OP_IMM:
                target = int(operands[0].imm)
                if target in symbols:
                    comments.append(compact_symbol_name(symbols[target]))

        rendered = f"  0x{instruction.address:08x}: {instruction.mnemonic:8s} {instruction.op_str}"
        if comments:
            rendered += "  ; " + " | ".join(comments)
        output.append(rendered)

    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument(
        "--function",
        action="append",
        required=True,
        help="Mangled or compact function-name substring; may be repeated.",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with args.elf.open("rb") as stream:
        elf = ELFFile(stream)
        if elf["e_machine"] != "EM_AARCH64":
            raise SystemExit(f"expected AArch64 ELF, got {elf['e_machine']}")
        symbols, functions = build_symbol_map(elf)
        selected = find_functions(functions, args.function)
        if not selected:
            raise SystemExit("no matching functions")
        lines: list[str] = []
        for function in selected:
            if lines:
                lines.append("")
            lines.extend(disassemble_function(elf, symbols, function))

    result = "\n".join(lines) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result, encoding="utf-8", newline="\n")
    else:
        print(result, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
