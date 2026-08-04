#!/usr/bin/env python3

"""Summarize Ethernet/IPv4/UDP packets in classic PCAP files.

The script uses only the Python standard library so it can run on the X30 host
or in the transfer workspace without Scapy, tshark, or tcpdump.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import socket
import struct
from pathlib import Path
from typing import Iterable


COMMAND_LABELS = {
    0x00000122: "physical_omega_x1000",
    0x00000123: "physical_vx_x1000",
    0x00000124: "physical_vy_x1000",
    0x00000150: "navigation_twist_3xf64",
    0x31040001: "navigation_heartbeat",
}


# 去除 classic-PCAP、Ethernet、IPv4 和 UDP 封装后的标准化数据包记录。
# 保留原始时间戳与四元组，供流量和频率检查使用。
@dataclasses.dataclass(frozen=True)
class UdpPacket:
    timestamp: float
    source_ip: str
    source_port: int
    destination_ip: str
    destination_port: int
    payload: bytes


# PCAP magic 同时决定字节序和微秒/纳秒时间戳尺度。
def _pcap_format(magic: bytes) -> tuple[str, float]:
    formats = {
        b"\xd4\xc3\xb2\xa1": ("<", 1_000_000.0),
        b"\xa1\xb2\xc3\xd4": (">", 1_000_000.0),
        b"\x4d\x3c\xb2\xa1": ("<", 1_000_000_000.0),
        b"\xa1\xb2\x3c\x4d": (">", 1_000_000_000.0),
    }
    if magic not in formats:
        raise ValueError(f"unsupported PCAP magic: {magic.hex()}")
    return formats[magic]


# 只解析 classic PCAP 的 Ethernet/IPv4/UDP 帧。
# 不支持的链路或协议记录直接跳过，避免猜测影响汇总可信度。
def read_udp_packets(path: Path) -> list[UdpPacket]:
    data = path.read_bytes()
    if len(data) < 24:
        raise ValueError(f"{path}: truncated PCAP global header")

    endian, timestamp_scale = _pcap_format(data[:4])
    offset = 24
    packets: list[UdpPacket] = []

    while offset + 16 <= len(data):
        # 每条记录的长度界定实际捕获字节，即使原始帧大于配置的抓取快照。
        seconds, fraction, captured_length, _ = struct.unpack_from(
            f"{endian}IIII", data, offset
        )
        offset += 16
        frame = data[offset : offset + captured_length]
        offset += captured_length

        if len(frame) < 14:
            continue

        # 接受普通 Ethernet II 或单层 802.1Q VLAN，随后要求 IPv4。
        ether_type = struct.unpack("!H", frame[12:14])[0]
        ip_offset = 14
        if ether_type == 0x8100 and len(frame) >= 18:
            ether_type = struct.unpack("!H", frame[16:18])[0]
            ip_offset = 18
        if ether_type != 0x0800 or len(frame) < ip_offset + 20:
            continue

        # IPv4 IHL 用于跳过可选 IP 选项并定位 UDP；协议号 17 表示 UDP。
        ip_header_length = (frame[ip_offset] & 0x0F) * 4
        if ip_header_length < 20 or frame[ip_offset + 9] != 17:
            continue

        udp_offset = ip_offset + ip_header_length
        if len(frame) < udp_offset + 8:
            continue

        source_port, destination_port, udp_length, _ = struct.unpack(
            "!HHHH", frame[udp_offset : udp_offset + 8]
        )
        payload = frame[udp_offset + 8 : udp_offset + udp_length]
        timestamp = seconds + fraction / timestamp_scale

        packets.append(
            UdpPacket(
                timestamp=timestamp,
                source_ip=socket.inet_ntoa(frame[ip_offset + 12 : ip_offset + 16]),
                source_port=source_port,
                destination_ip=socket.inet_ntoa(frame[ip_offset + 16 : ip_offset + 20]),
                destination_port=destination_port,
                payload=payload,
            )
        )

    return packets


# X30 命令帧以 little-endian u32 命令码开头。
def command_code(payload: bytes) -> int | None:
    if len(payload) < 4:
        return None
    return struct.unpack("<I", payload[:4])[0]


# 报告流、payload 尺寸、命令值和平均频率，但不宣称机器人已执行；
# PCAP 只能证明观测到网络流量。
def summarize(path: Path) -> str:
    packets = read_udp_packets(path)
    lines = [f"===== {path.name} =====", f"UDP packets: {len(packets)}"]
    if not packets:
        return "\n".join(lines)

    duration = packets[-1].timestamp - packets[0].timestamp
    lines.append(f"Capture span: {duration:.6f} s")

    flow_counts = collections.Counter(
        (
            packet.source_ip,
            packet.source_port,
            packet.destination_ip,
            packet.destination_port,
        )
        for packet in packets
    )
    lines.append("Flows:")
    for flow, count in flow_counts.most_common():
        lines.append(f"  {flow[0]}:{flow[1]} -> {flow[2]}:{flow[3]}: {count}")

    length_counts = collections.Counter(len(packet.payload) for packet in packets)
    lines.append(f"UDP payload lengths: {dict(sorted(length_counts.items()))}")

    code_counts = collections.Counter(command_code(packet.payload) for packet in packets)
    lines.append("Little-endian command codes:")
    for code, count in code_counts.most_common():
        label = "short-payload" if code is None else f"0x{code:08x}"
        if code in COMMAND_LABELS:
            label += f" ({COMMAND_LABELS[code]})"
        lines.append(f"  {label}: {count}")

    payload_counts = collections.Counter(packet.payload for packet in packets)
    lines.append(f"Unique UDP payloads: {len(payload_counts)}")
    for payload, count in payload_counts.most_common(20):
        interpretation = ""
        # 只解析尺寸和头部已知的数据包布局。
        # 未知 payload 保留 raw hex，供后续协议分析。
        if len(payload) == 12:
            word0, word1, word2 = struct.unpack("<III", payload)
            signed_word1 = struct.unpack("<i", payload[4:8])[0]
            float1 = struct.unpack("<f", payload[4:8])[0]
            interpretation = (
                f" words=({word0}, {word1}, {word2})"
                f" word1_as_int32={signed_word1}"
                f" word1_as_float={float1:.9g}"
            )
            if word0 in {0x122, 0x123, 0x124}:
                interpretation += f" physical_value={signed_word1 / 1000.0:.6g}"
        elif len(payload) == 36:
            code, data_size, data_type = struct.unpack("<III", payload[:12])
            if code == 0x150 and data_size == 24 and data_type == 1:
                vx, vy, omega = struct.unpack("<ddd", payload[12:36])
                interpretation = (
                    f" navigation_twist=(vx={vx:.9g}, vy={vy:.9g},"
                    f" omega={omega:.9g})"
                )
        lines.append(
            f"  count={count} length={len(payload)} hex={payload.hex()}"
            f"{interpretation}"
        )

    code_times: dict[tuple[int, int | None], list[float]] = collections.defaultdict(list)
    start = packets[0].timestamp
    for packet in packets:
        code_times[(packet.source_port, command_code(packet.payload))].append(
            packet.timestamp - start
        )

    lines.append("Code timing:")
    for (source_port, code), times in sorted(
        code_times.items(), key=lambda item: (item[0][0], item[0][1] or -1)
    ):
        label = "short-payload" if code is None else f"0x{code:08x}"
        if code in COMMAND_LABELS:
            label += f" ({COMMAND_LABELS[code]})"
        span = times[-1] - times[0]
        interval_hz = 0.0
        if len(times) > 1 and span > 0.0:
            interval_hz = (len(times) - 1) / span
        lines.append(
            f"  source_port={source_port} code={label} count={len(times)}"
            f" first={times[0]:.6f}s last={times[-1]:.6f}s"
            f" average_hz={interval_hz:.3f}"
        )

    return "\n".join(lines)


# 以确定顺序展开目录参数，同时保留显式文件参数。
def pcap_paths(arguments: Iterable[str]) -> list[Path]:
    paths: list[Path] = []
    for argument in arguments:
        path = Path(argument)
        if path.is_dir():
            paths.extend(sorted(path.glob("*.pcap")))
        else:
            paths.append(path)
    return paths


# 命令行层只负责选择输入并输出汇总；解析保留为纯函数，
# 使抓包文件也能通过程序化测试。
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="PCAP files or directories")
    args = parser.parse_args()

    paths = pcap_paths(args.paths)
    if not paths:
        parser.error("no PCAP files found")

    for index, path in enumerate(paths):
        if index:
            print()
        print(summarize(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
