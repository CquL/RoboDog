#!/usr/bin/env python3
"""Build a deterministic regression summary from replay-test JSONL output."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


SCHEMA_NAME = "x30_plane_seg_replay_summary_v1"
DEGENERATE_SIZE_EPSILON = 1.0e-6
FRAME_KEYS = {
    "case",
    "stamp_ns",
    "selected_index",
    "retained",
    "candidate_count",
    "factory_group_count",
    "candidates",
}
CANDIDATE_KEYS = {
    "type",
    "size",
    "translation",
    "quaternion_xyzw",
    "hull",
}


class SummaryError(ValueError):
    """Raised when replay output does not satisfy the regression contract."""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _reject_json_constant(value: str) -> None:
    raise SummaryError(f"non-finite JSON constant is not allowed: {value}")


def _require_exact_keys(
    value: Dict[str, Any], expected: set[str], context: str
) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise SummaryError(f"{context} has invalid keys ({'; '.join(details)})")


def _require_nonnegative_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SummaryError(f"{context} must be a non-negative integer")
    return value


def _require_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SummaryError(f"{context} must be an integer")
    return value


def _require_finite_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SummaryError(f"{context} must be a finite number")
    try:
        converted = float(value)
    except OverflowError as error:
        raise SummaryError(f"{context} must be representable as a finite number") from error
    if not math.isfinite(converted):
        raise SummaryError(f"{context} must be finite")
    return converted


def _require_vector(value: Any, length: int, context: str) -> None:
    if not isinstance(value, list) or len(value) != length:
        raise SummaryError(f"{context} must be an array of length {length}")
    for index, item in enumerate(value):
        _require_finite_number(item, f"{context}[{index}]")


def _validate_candidate(candidate: Any, line_number: int, index: int) -> bool:
    context = f"line {line_number} candidate {index}"
    if not isinstance(candidate, dict):
        raise SummaryError(f"{context} must be an object")
    _require_exact_keys(candidate, CANDIDATE_KEYS, context)
    _require_int(candidate["type"], f"{context}.type")
    _require_vector(candidate["size"], 3, f"{context}.size")
    _require_vector(candidate["translation"], 3, f"{context}.translation")
    _require_vector(
        candidate["quaternion_xyzw"], 4, f"{context}.quaternion_xyzw"
    )
    hull = candidate["hull"]
    if not isinstance(hull, list):
        raise SummaryError(f"{context}.hull must be an array")
    for point_index, point in enumerate(hull):
        _require_vector(point, 3, f"{context}.hull[{point_index}]")
    return (
        float(candidate["size"][0]) <= DEGENERATE_SIZE_EPSILON
        or float(candidate["size"][1]) <= DEGENERATE_SIZE_EPSILON
        or len(hull) < 3
    )


def _canonical_json_line(frame: Dict[str, Any]) -> bytes:
    return json.dumps(
        frame,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")


def _parse_frame(
    line: str, line_number: int
) -> Tuple[Dict[str, Any], Dict[str, Any], int]:
    if not line.strip():
        raise SummaryError(f"line {line_number} is blank")
    try:
        frame = json.loads(line, parse_constant=_reject_json_constant)
    except SummaryError:
        raise
    except (json.JSONDecodeError, UnicodeError) as error:
        raise SummaryError(f"line {line_number} is not valid JSON: {error}") from error
    if not isinstance(frame, dict):
        raise SummaryError(f"line {line_number} must contain a JSON object")
    _require_exact_keys(frame, FRAME_KEYS, f"line {line_number}")

    case_name = frame["case"]
    if not isinstance(case_name, str) or not case_name.strip():
        raise SummaryError(f"line {line_number}.case must be a non-empty string")
    stamp_ns = _require_nonnegative_int(frame["stamp_ns"], f"line {line_number}.stamp_ns")
    selected_index = _require_nonnegative_int(
        frame["selected_index"], f"line {line_number}.selected_index"
    )
    retained = _require_nonnegative_int(
        frame["retained"], f"line {line_number}.retained"
    )
    candidate_count = _require_nonnegative_int(
        frame["candidate_count"], f"line {line_number}.candidate_count"
    )
    factory_group_count = _require_nonnegative_int(
        frame["factory_group_count"],
        f"line {line_number}.factory_group_count",
    )

    candidates = frame["candidates"]
    if not isinstance(candidates, list):
        raise SummaryError(f"line {line_number}.candidates must be an array")
    if candidate_count != len(candidates):
        raise SummaryError(
            f"line {line_number}.candidate_count={candidate_count} does not match "
            f"candidates length {len(candidates)}"
        )
    degenerate_count = sum(
        _validate_candidate(candidate, line_number, candidate_index)
        for candidate_index, candidate in enumerate(candidates)
    )

    canonical_hash = _sha256(_canonical_json_line(frame))
    light_frame = {
        "case": case_name,
        "stamp_ns": stamp_ns,
        "selected_index": selected_index,
        "retained": retained,
        "candidate_count": candidate_count,
        "factory_group_count": factory_group_count,
        "canonical_line_sha256": canonical_hash,
    }
    return frame, light_frame, degenerate_count


def build_summary(input_bytes: bytes, source_pack_bytes: Optional[bytes] = None) -> Dict[str, Any]:
    try:
        input_text = input_bytes.decode("utf-8")
    except UnicodeDecodeError as error:
        raise SummaryError(f"input JSONL is not valid UTF-8: {error}") from error

    lines = input_text.splitlines()
    if not lines:
        raise SummaryError("input JSONL contains no frames")

    light_frames: List[Dict[str, Any]] = []
    frame_keys = set()
    degenerate_counts: Dict[Tuple[str, int, int], int] = {}
    for line_number, line in enumerate(lines, start=1):
        _, light_frame, degenerate_count = _parse_frame(line, line_number)
        frame_key = (
            light_frame["case"],
            light_frame["stamp_ns"],
            light_frame["selected_index"],
        )
        if frame_key in frame_keys:
            raise SummaryError(
                "duplicate frame key at line "
                f"{line_number}: case={frame_key[0]!r}, stamp_ns={frame_key[1]}, "
                f"selected_index={frame_key[2]}"
            )
        frame_keys.add(frame_key)
        degenerate_counts[frame_key] = degenerate_count
        light_frames.append(light_frame)

    light_frames.sort(
        key=lambda frame: (
            frame["case"],
            frame["stamp_ns"],
            frame["selected_index"],
        )
    )
    retained_counts = [frame["retained"] for frame in light_frames]
    candidate_counts = [frame["candidate_count"] for frame in light_frames]
    factory_counts = [frame["factory_group_count"] for frame in light_frames]
    histogram = Counter(candidate_counts)
    degenerate_frames = []
    for frame in light_frames:
        frame_key = (frame["case"], frame["stamp_ns"], frame["selected_index"])
        count = degenerate_counts[frame_key]
        if count:
            degenerate_frames.append(
                {
                    "case": frame["case"],
                    "stamp_ns": frame["stamp_ns"],
                    "selected_index": frame["selected_index"],
                    "count": count,
                }
            )

    summary: Dict[str, Any] = {
        "schema": SCHEMA_NAME,
        "input_jsonl_sha256": _sha256(input_bytes),
        "frame_count": len(light_frames),
        "retained": {
            "min": min(retained_counts),
            "max": max(retained_counts),
        },
        "candidates": {
            "min": min(candidate_counts),
            "max": max(candidate_counts),
            "total": sum(candidate_counts),
            "histogram": {
                str(count): histogram[count] for count in sorted(histogram)
            },
        },
        "factory_groups": {
            "min": min(factory_counts),
            "max": max(factory_counts),
            "total": sum(factory_counts),
        },
        "count_equal_factory": sum(
            candidate == factory
            for candidate, factory in zip(candidate_counts, factory_counts)
        ),
        "degenerate_candidate_count": sum(degenerate_counts.values()),
        "degenerate_frames": degenerate_frames,
        "frames": light_frames,
    }
    if source_pack_bytes is not None:
        summary["source_pack"] = {
            "sha256": _sha256(source_pack_bytes),
            "size_bytes": len(source_pack_bytes),
        }
    return summary


def render_summary(summary: Dict[str, Any]) -> bytes:
    text = json.dumps(
        summary,
        sort_keys=True,
        indent=2,
        ensure_ascii=True,
        allow_nan=False,
    )
    return (text + "\n").encode("ascii")


def summarize_file(
    input_path: Path, output_path: Path, source_pack_path: Optional[Path] = None
) -> Dict[str, Any]:
    input_bytes = input_path.read_bytes()
    source_pack_bytes = source_pack_path.read_bytes() if source_pack_path else None
    summary = build_summary(input_bytes, source_pack_bytes)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(render_summary(summary))
    return summary


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize x30_plane_seg_core replay JSONL deterministically."
    )
    parser.add_argument("--input", required=True, type=Path, help="Replay JSONL path")
    parser.add_argument("--output", required=True, type=Path, help="Summary JSON path")
    parser.add_argument(
        "--source-pack",
        type=Path,
        help="Optional .x30rpl source pack recorded by hash and byte size",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        summarize_file(args.input, args.output, args.source_pack)
    except (OSError, SummaryError) as error:
        print(f"x30 replay summary failure: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
