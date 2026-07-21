#!/usr/bin/env python3
"""Analyze X30 replay factory/candidate geometry without network or robot I/O.

This is an offline regression analyzer.  It reports observable geometry and does
not claim to reconstruct the final X30 quadrangle algorithm.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import struct
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


SCHEMA_NAME = "x30_quadrangle_parity_analysis_v1"
MAGIC = b"X30RPLY\0"
SCHEMA_MAJOR = 1
SCHEMA_MINOR = 0
HEADER_BYTES = 128
HEADER_FLAGS = 3
FRAME_RECORD_BYTES = 384
SCHEMA_TOLERANCE = 1.0e-5
UINT32_MAX = (1 << 32) - 1

HAS_EXACT_TF = 1
EXPECT_MISSING_TF = 2
HAS_FACTORY_ORACLE = 4
HAS_CORE_ORACLE = 8
RUN_CORE = 16
KNOWN_FRAME_FLAGS = (
    HAS_EXACT_TF
    | EXPECT_MISSING_TF
    | HAS_FACTORY_ORACLE
    | HAS_CORE_ORACLE
    | RUN_CORE
)

HEADER_STRUCT = struct.Struct("<8sHHIIIIIQQQ32s32sff")
FRAME_STRUCT = struct.Struct("<64sQ6I17d9fBBH4I8Q32s")
FLOAT32_STRUCT = struct.Struct("<f")
POINT_STRUCT = struct.Struct("<fff")

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

Point = tuple[float, float, float]
FrameKey = tuple[str, int, int]


class ParityAnalysisError(ValueError):
    """Raised when an input violates the offline regression contract."""


@dataclass(frozen=True)
class ReplayHeader:
    source_manifest_sha256: str
    body_sha256: str
    absolute_tolerance: float
    relative_tolerance: float


@dataclass(frozen=True)
class ReplayFrame:
    case: str
    stamp_ns: int
    selected_index: int
    expected_retained_count: int
    factory_points: tuple[Point, ...]

    @property
    def key(self) -> FrameKey:
        return (self.case, self.stamp_ns, self.selected_index)


@dataclass(frozen=True)
class ReplayPack:
    header: ReplayHeader
    sha256: str
    size_bytes: int
    frames: tuple[ReplayFrame, ...]


@dataclass(frozen=True)
class Candidate:
    type: int
    size: Point
    translation: Point
    quaternion_xyzw: tuple[float, float, float, float]
    hull: tuple[Point, ...]


@dataclass(frozen=True)
class ReplayOutputFrame:
    case: str
    stamp_ns: int
    selected_index: int
    retained: int
    factory_group_count: int
    candidates: tuple[Candidate, ...]

    @property
    def key(self) -> FrameKey:
        return (self.case, self.stamp_ns, self.selected_index)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _f32(value: float) -> float:
    return FLOAT32_STRUCT.unpack(FLOAT32_STRUCT.pack(value))[0]


def _finite(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParityAnalysisError(f"{context} must be a finite number")
    try:
        result = float(value)
    except OverflowError as error:
        raise ParityAnalysisError(f"{context} must be finite") from error
    if not math.isfinite(result):
        raise ParityAnalysisError(f"{context} must be finite")
    return result


def _nonnegative_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ParityAnalysisError(f"{context} must be a non-negative integer")
    return value


def _integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ParityAnalysisError(f"{context} must be an integer")
    return value


def _vector(value: Any, length: int, context: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ParityAnalysisError(f"{context} must be an array of length {length}")
    return tuple(_finite(item, f"{context}[{index}]") for index, item in enumerate(value))


def _normalizable(values: Sequence[float]) -> bool:
    scale = max(abs(value) for value in values)
    if not math.isfinite(scale) or scale == 0.0:
        return False
    scaled_norm_squared = sum((value / scale) ** 2 for value in values)
    return math.isfinite(scaled_norm_squared) and scaled_norm_squared > 0.0


def _decode_case_name(raw: bytes, frame_index: int) -> str:
    context = f"frame {frame_index}: case_name"
    try:
        terminator = raw.index(0)
    except ValueError as error:
        raise ParityAnalysisError(f"{context} is not NUL terminated") from error
    if terminator == 0:
        raise ParityAnalysisError(f"{context} is empty")
    if any(raw[terminator:]):
        raise ParityAnalysisError(f"{context} has non-zero bytes after its terminator")
    try:
        return raw[:terminator].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ParityAnalysisError(f"{context} is not valid UTF-8") from error


def _validate_pose(values: Sequence[float], quaternion: Sequence[float], context: str) -> None:
    if not all(math.isfinite(value) for value in (*values, *quaternion)):
        raise ParityAnalysisError(f"{context} must be finite")
    if not _normalizable(quaternion):
        raise ParityAnalysisError(f"{context} quaternion must be normalizable")


def _validate_blob(
    offset: int,
    byte_count: int,
    expected_bytes: int,
    payload_offset: int,
    file_bytes: int,
    name: str,
    ranges: list[tuple[int, int, str]],
) -> None:
    if byte_count != expected_bytes:
        raise ParityAnalysisError(
            f"{name} byte count is {byte_count}, expected {expected_bytes}"
        )
    if expected_bytes == 0:
        if offset != 0:
            raise ParityAnalysisError(f"{name} has a non-zero offset for an empty blob")
        return
    if offset < payload_offset:
        raise ParityAnalysisError(f"{name} starts before payload_offset")
    end = offset + byte_count
    if end > file_bytes:
        raise ParityAnalysisError(f"{name} extends past file_bytes")
    ranges.append((offset, end, name))


def parse_replay_pack(data: bytes) -> ReplayPack:
    """Strictly parse one schema-v1 x30rpl pack and decode factory XYZ groups."""

    if not isinstance(data, bytes):
        raise ParityAnalysisError("replay pack must be bytes")
    if len(data) < HEADER_BYTES:
        raise ParityAnalysisError("replay pack is shorter than the 128-byte header")
    try:
        header_values = HEADER_STRUCT.unpack_from(data)
    except struct.error as error:
        raise ParityAnalysisError(f"unable to decode replay header: {error}") from error

    (
        magic,
        major,
        minor,
        header_bytes,
        header_flags,
        frame_count,
        frame_record_bytes,
        reserved,
        frame_table_offset,
        payload_offset,
        file_bytes,
        source_manifest_digest,
        body_digest,
        absolute_tolerance,
        relative_tolerance,
    ) = header_values

    if magic != MAGIC:
        raise ParityAnalysisError("header magic is not X30RPLY\\0")
    if (major, minor) != (SCHEMA_MAJOR, SCHEMA_MINOR):
        raise ParityAnalysisError("only replay schema 1.0 is supported")
    if header_bytes != HEADER_BYTES or frame_record_bytes != FRAME_RECORD_BYTES:
        raise ParityAnalysisError("fixed header or frame record byte count is invalid")
    if header_flags != HEADER_FLAGS:
        raise ParityAnalysisError(
            "header flags must declare little-endian IEC 60559 payloads"
        )
    if reserved != 0:
        raise ParityAnalysisError("header reserved field is non-zero")
    if frame_count == 0:
        raise ParityAnalysisError("header frame_count must be non-zero")
    if frame_table_offset != HEADER_BYTES:
        raise ParityAnalysisError("header frame_table_offset must be 128")
    if file_bytes != len(data):
        raise ParityAnalysisError("header file_bytes does not match physical file size")
    table_end = frame_table_offset + frame_count * frame_record_bytes
    if table_end > file_bytes or payload_offset < table_end or payload_offset > file_bytes:
        raise ParityAnalysisError("frame table or payload offset is outside its allowed range")
    if (
        absolute_tolerance != _f32(SCHEMA_TOLERANCE)
        or relative_tolerance != _f32(SCHEMA_TOLERANCE)
    ):
        raise ParityAnalysisError("schema-v1 tolerances must both equal 1e-5")
    actual_body_digest = hashlib.sha256(data[HEADER_BYTES:]).digest()
    if actual_body_digest != body_digest:
        raise ParityAnalysisError(
            "SHA256 of bytes [128,file_bytes) does not match body_sha256"
        )

    pending_frames: list[tuple[ReplayFrame, tuple[int, int], tuple[int, int], tuple[int, int], bytes]] = []
    ranges: list[tuple[int, int, str]] = []
    seen_keys: set[FrameKey] = set()

    for frame_index in range(frame_count):
        record_offset = HEADER_BYTES + frame_index * FRAME_RECORD_BYTES
        try:
            values = FRAME_STRUCT.unpack_from(data, record_offset)
        except struct.error as error:
            raise ParityAnalysisError(
                f"frame {frame_index} record is truncated: {error}"
            ) from error

        case = _decode_case_name(values[0], frame_index)
        prefix = f"frame {frame_index} ({case})"
        stamp_ns = values[1]
        selected_index, flags, size_x, size_y, outer_start, inner_start = values[2:8]
        resolution, length_x, length_y = values[8:11]
        center = values[11:14]
        orientation = values[14:18]
        world_to_base_translation = values[18:21]
        world_to_base_rotation = values[21:25]
        sensor_origin = values[25:28]
        sensor_look = values[28:31]
        accessibility_threshold, downsample_resolution, max_angle = values[31:34]
        factory_mode_9, debug, reserved_16 = values[34:37]
        expected_retained_count, expected_core_count, factory_point_count, reserved_32 = values[37:41]
        elevation_ref = (values[41], values[42])
        accessibility_ref = (values[43], values[44])
        factory_ref = (values[45], values[46])
        core_ref = (values[47], values[48])
        frame_input_digest = values[49]

        if flags & ~KNOWN_FRAME_FLAGS:
            raise ParityAnalysisError(f"{prefix}: unknown frame flag bits are set")
        if not flags & RUN_CORE:
            raise ParityAnalysisError(f"{prefix}: RUN_CORE is required")
        if flags & HAS_CORE_ORACLE:
            raise ParityAnalysisError(
                f"{prefix}: HAS_CORE_ORACLE is unsupported in schema v1"
            )
        if expected_core_count != UINT32_MAX or core_ref != (0, 0):
            raise ParityAnalysisError(f"{prefix}: core oracle fields must be absent")
        has_exact_tf = bool(flags & HAS_EXACT_TF)
        expects_missing_tf = bool(flags & EXPECT_MISSING_TF)
        if has_exact_tf == expects_missing_tf:
            raise ParityAnalysisError(
                f"{prefix}: exactly one of HAS_EXACT_TF and EXPECT_MISSING_TF is required"
            )
        if size_x == 0 or size_y == 0:
            raise ParityAnalysisError(f"{prefix}: grid dimensions must be non-zero")
        if outer_start >= size_x or inner_start >= size_y:
            raise ParityAnalysisError(f"{prefix}: grid start index is outside its dimension")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (resolution, length_x, length_y)
        ):
            raise ParityAnalysisError(
                f"{prefix}: grid resolution and lengths must be finite and positive"
            )
        expected_length_x = size_x * resolution
        expected_length_y = size_y * resolution
        if not math.isclose(length_x, expected_length_x, rel_tol=1.0e-9, abs_tol=1.0e-9):
            raise ParityAnalysisError(f"{prefix}: length_x does not match size_x * resolution")
        if not math.isclose(length_y, expected_length_y, rel_tol=1.0e-9, abs_tol=1.0e-9):
            raise ParityAnalysisError(f"{prefix}: length_y does not match size_y * resolution")
        _validate_pose(center, orientation, f"{prefix}: grid pose")
        _validate_pose(
            world_to_base_translation,
            world_to_base_rotation,
            f"{prefix}: world-to-base pose",
        )
        if expects_missing_tf and (
            tuple(world_to_base_translation) != (0.0, 0.0, 0.0)
            or tuple(world_to_base_rotation) != (0.0, 0.0, 0.0, 1.0)
        ):
            raise ParityAnalysisError(
                f"{prefix}: missing TF must preserve zero translation and identity rotation"
            )
        if not all(math.isfinite(value) for value in (*sensor_origin, *sensor_look)):
            raise ParityAnalysisError(f"{prefix}: sensor pose must be finite")
        if not _normalizable(sensor_look):
            raise ParityAnalysisError(f"{prefix}: sensor look direction must be non-zero")
        if tuple(sensor_origin) != (0.0, 0.0, 0.0) or tuple(sensor_look) != (1.0, 0.0, 0.0):
            raise ParityAnalysisError(f"{prefix}: schema-v1 sensor pose is invalid")
        if (
            accessibility_threshold != _f32(0.9)
            or downsample_resolution != _f32(0.01)
            or max_angle != _f32(45.0)
        ):
            raise ParityAnalysisError(f"{prefix}: core configuration constants are invalid")
        if factory_mode_9 not in (0, 1) or debug not in (0, 1):
            raise ParityAnalysisError(f"{prefix}: boolean fields must be encoded as 0 or 1")
        if factory_mode_9 or debug:
            raise ParityAnalysisError(f"{prefix}: schema-v1 requires factory_mode_9=0 and debug=0")
        if reserved_16 != 0 or reserved_32 != 0:
            raise ParityAnalysisError(f"{prefix}: reserved frame fields must be zero")

        cell_count = size_x * size_y
        if expected_retained_count > cell_count:
            raise ParityAnalysisError(
                f"{prefix}: expected retained count exceeds input cell count"
            )
        if not flags & HAS_FACTORY_ORACLE and (
            factory_point_count != 0 or factory_ref != (0, 0)
        ):
            raise ParityAnalysisError(
                f"{prefix}: factory points are present without HAS_FACTORY_ORACLE"
            )
        if factory_point_count % 4 != 0:
            raise ParityAnalysisError(
                f"{prefix}: factory point count is not divisible by four"
            )

        layer_bytes = cell_count * FLOAT32_STRUCT.size
        factory_bytes = factory_point_count * POINT_STRUCT.size
        _validate_blob(
            *elevation_ref,
            layer_bytes,
            payload_offset,
            file_bytes,
            f"{prefix}: elevation",
            ranges,
        )
        _validate_blob(
            *accessibility_ref,
            layer_bytes,
            payload_offset,
            file_bytes,
            f"{prefix}: accessibility",
            ranges,
        )
        _validate_blob(
            *factory_ref,
            factory_bytes,
            payload_offset,
            file_bytes,
            f"{prefix}: factory XYZ",
            ranges,
        )

        key = (case, stamp_ns, selected_index)
        if key in seen_keys:
            raise ParityAnalysisError(f"duplicate replay frame key: {key!r}")
        seen_keys.add(key)
        placeholder = ReplayFrame(case, stamp_ns, selected_index, expected_retained_count, ())
        pending_frames.append(
            (placeholder, elevation_ref, accessibility_ref, factory_ref, frame_input_digest)
        )

    ranges.sort(key=lambda item: (item[0], item[1]))
    covered_until = payload_offset
    for begin, end, name in ranges:
        if begin != covered_until:
            raise ParityAnalysisError(
                f"{name} does not immediately follow the preceding payload blob"
            )
        covered_until = end
    if covered_until != file_bytes:
        raise ParityAnalysisError("payload blobs do not cover the complete declared payload")

    frames: list[ReplayFrame] = []
    for frame_index, pending in enumerate(pending_frames):
        placeholder, elevation_ref, accessibility_ref, factory_ref, expected_input_digest = pending
        blob_bytes = b"".join(
            data[offset : offset + byte_count]
            for offset, byte_count in (elevation_ref, accessibility_ref, factory_ref)
        )
        if hashlib.sha256(blob_bytes).digest() != expected_input_digest:
            raise ParityAnalysisError(
                f"frame {frame_index} ({placeholder.case}): frame_input_sha256 mismatch"
            )
        factory_offset, factory_bytes = factory_ref
        factory_points = tuple(
            POINT_STRUCT.unpack_from(data, factory_offset + offset)
            for offset in range(0, factory_bytes, POINT_STRUCT.size)
        )
        if not all(math.isfinite(value) for point in factory_points for value in point):
            raise ParityAnalysisError(
                f"frame {frame_index} ({placeholder.case}): factory XYZ is non-finite"
            )
        frames.append(
            ReplayFrame(
                case=placeholder.case,
                stamp_ns=placeholder.stamp_ns,
                selected_index=placeholder.selected_index,
                expected_retained_count=placeholder.expected_retained_count,
                factory_points=factory_points,
            )
        )

    return ReplayPack(
        header=ReplayHeader(
            source_manifest_sha256=source_manifest_digest.hex(),
            body_sha256=body_digest.hex(),
            absolute_tolerance=absolute_tolerance,
            relative_tolerance=relative_tolerance,
        ),
        sha256=_sha256(data),
        size_bytes=len(data),
        frames=tuple(frames),
    )


def _reject_json_constant(value: str) -> None:
    raise ParityAnalysisError(f"non-finite JSON constant is not allowed: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ParityAnalysisError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _exact_keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    actual = set(value)
    if actual == expected:
        return
    details = []
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing:
        details.append("missing=" + ",".join(missing))
    if extra:
        details.append("extra=" + ",".join(extra))
    raise ParityAnalysisError(f"{context} has invalid keys ({'; '.join(details)})")


def parse_replay_jsonl(data: bytes) -> tuple[ReplayOutputFrame, ...]:
    """Strictly parse replay_fixture_test's canonical JSONL schema."""

    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ParityAnalysisError(f"input JSONL is not valid UTF-8: {error}") from error
    lines = text.splitlines()
    if not lines:
        raise ParityAnalysisError("input JSONL contains no frames")

    frames: list[ReplayOutputFrame] = []
    seen_keys: set[FrameKey] = set()
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            raise ParityAnalysisError(f"line {line_number} is blank")
        try:
            value = json.loads(
                line,
                parse_constant=_reject_json_constant,
                object_pairs_hook=_unique_object,
            )
        except ParityAnalysisError as error:
            raise ParityAnalysisError(f"line {line_number}: {error}") from error
        except (json.JSONDecodeError, UnicodeError) as error:
            raise ParityAnalysisError(
                f"line {line_number} is not valid JSON: {error}"
            ) from error
        if not isinstance(value, dict):
            raise ParityAnalysisError(f"line {line_number} must contain a JSON object")
        context = f"line {line_number}"
        _exact_keys(value, FRAME_KEYS, context)

        case = value["case"]
        if not isinstance(case, str) or not case.strip():
            raise ParityAnalysisError(f"{context}.case must be a non-empty string")
        stamp_ns = _nonnegative_int(value["stamp_ns"], f"{context}.stamp_ns")
        selected_index = _nonnegative_int(
            value["selected_index"], f"{context}.selected_index"
        )
        retained = _nonnegative_int(value["retained"], f"{context}.retained")
        candidate_count = _nonnegative_int(
            value["candidate_count"], f"{context}.candidate_count"
        )
        factory_group_count = _nonnegative_int(
            value["factory_group_count"], f"{context}.factory_group_count"
        )
        raw_candidates = value["candidates"]
        if not isinstance(raw_candidates, list):
            raise ParityAnalysisError(f"{context}.candidates must be an array")
        if candidate_count != len(raw_candidates):
            raise ParityAnalysisError(
                f"{context}.candidate_count={candidate_count} does not match "
                f"candidates length {len(raw_candidates)}"
            )

        candidates: list[Candidate] = []
        for candidate_index, raw_candidate in enumerate(raw_candidates):
            candidate_context = f"{context}.candidates[{candidate_index}]"
            if not isinstance(raw_candidate, dict):
                raise ParityAnalysisError(f"{candidate_context} must be an object")
            _exact_keys(raw_candidate, CANDIDATE_KEYS, candidate_context)
            candidate_type = _integer(raw_candidate["type"], f"{candidate_context}.type")
            size = _vector(raw_candidate["size"], 3, f"{candidate_context}.size")
            translation = _vector(
                raw_candidate["translation"], 3, f"{candidate_context}.translation"
            )
            quaternion = _vector(
                raw_candidate["quaternion_xyzw"],
                4,
                f"{candidate_context}.quaternion_xyzw",
            )
            raw_hull = raw_candidate["hull"]
            if not isinstance(raw_hull, list):
                raise ParityAnalysisError(f"{candidate_context}.hull must be an array")
            hull = tuple(
                _vector(point, 3, f"{candidate_context}.hull[{point_index}]")
                for point_index, point in enumerate(raw_hull)
            )
            candidates.append(
                Candidate(
                    type=candidate_type,
                    size=size,  # type: ignore[arg-type]
                    translation=translation,  # type: ignore[arg-type]
                    quaternion_xyzw=quaternion,  # type: ignore[arg-type]
                    hull=hull,  # type: ignore[arg-type]
                )
            )

        frame = ReplayOutputFrame(
            case=case,
            stamp_ns=stamp_ns,
            selected_index=selected_index,
            retained=retained,
            factory_group_count=factory_group_count,
            candidates=tuple(candidates),
        )
        if frame.key in seen_keys:
            raise ParityAnalysisError(f"duplicate JSONL frame key: {frame.key!r}")
        seen_keys.add(frame.key)
        frames.append(frame)
    return tuple(frames)


def _subtract(left: Point, right: Point) -> Point:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _cross(left: Point, right: Point) -> Point:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left: Point, right: Point) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Point) -> float:
    return math.sqrt(_dot(vector, vector))


def _triangle_area(first: Point, second: Point, third: Point) -> float:
    return 0.5 * _norm(_cross(_subtract(second, first), _subtract(third, first)))


def _quadrangle_area(points: Sequence[Point]) -> float:
    return _triangle_area(points[0], points[1], points[2]) + _triangle_area(
        points[0], points[2], points[3]
    )


def _polygon_area(points: Sequence[Point]) -> float:
    if len(points) < 3:
        return 0.0
    area_vector = (0.0, 0.0, 0.0)
    for first, second in zip(points, (*points[1:], points[0])):
        edge_cross = _cross(first, second)
        area_vector = (
            area_vector[0] + edge_cross[0],
            area_vector[1] + edge_cross[1],
            area_vector[2] + edge_cross[2],
        )
    return 0.5 * _norm(area_vector)


def _signed_xy_area(points: Sequence[Point]) -> float:
    return 0.5 * sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, (*points[1:], points[0]))
    )


def _close(left: float, right: float, absolute: float, relative: float) -> bool:
    return math.isclose(left, right, abs_tol=absolute, rel_tol=relative)


def _point_error(left: Point, right: Point) -> float:
    return max(abs(a - b) for a, b in zip(left, right))


def _points_close(
    left: Point, right: Point, absolute: float, relative: float
) -> bool:
    return all(_close(a, b, absolute, relative) for a, b in zip(left, right))


def _sequence_match(
    left: Sequence[Point],
    right: Sequence[Point],
    absolute: float,
    relative: float,
) -> tuple[bool, float]:
    if len(left) != len(right):
        return False, math.inf
    maximum_error = max(
        (_point_error(first, second) for first, second in zip(left, right)),
        default=0.0,
    )
    return (
        all(
            _points_close(first, second, absolute, relative)
            for first, second in zip(left, right)
        ),
        maximum_error,
    )


def _planarity(
    points: Sequence[Point],
    absolute_tolerance: float,
    relative_tolerance: float,
    area_tolerance: float,
) -> tuple[bool, bool, float]:
    best_normal: Optional[Point] = None
    best_origin: Optional[Point] = None
    best_norm = 0.0
    for first_index, second_index, third_index in itertools.combinations(range(4), 3):
        origin = points[first_index]
        normal = _cross(
            _subtract(points[second_index], origin),
            _subtract(points[third_index], origin),
        )
        normal_norm = _norm(normal)
        if normal_norm > best_norm:
            best_normal = normal
            best_origin = origin
            best_norm = normal_norm
    if best_normal is None or best_origin is None or best_norm <= 2.0 * area_tolerance:
        return True, False, 0.0

    unit_normal = tuple(value / best_norm for value in best_normal)
    maximum_error = max(
        abs(_dot(unit_normal, _subtract(point, best_origin))) for point in points
    )
    scale = max(
        1.0,
        max(
            _norm(_subtract(points[right], points[left]))
            for left, right in itertools.combinations(range(4), 2)
        ),
    )
    limit = absolute_tolerance + relative_tolerance * scale
    return maximum_error <= limit, True, maximum_error


def _numeric_summary(values: Sequence[float], zero_count: int) -> dict[str, Any]:
    if not values:
        return {
            "minimum": None,
            "maximum": None,
            "total": 0.0,
            "zero_count": zero_count,
        }
    return {
        "minimum": min(values),
        "maximum": max(values),
        "total": math.fsum(values),
        "zero_count": zero_count,
    }


def _pair_row(left: int, right: int, maximum_error: float, **extra: Any) -> dict[str, Any]:
    row: dict[str, Any] = {
        "left_group_index": left,
        "right_group_index": right,
        "maximum_coordinate_error_m": maximum_error,
    }
    row.update(extra)
    return row


def _factory_analysis(
    points: Sequence[Point],
    absolute_tolerance: float,
    relative_tolerance: float,
    area_tolerance: float,
) -> dict[str, Any]:
    groups = [tuple(points[index : index + 4]) for index in range(0, len(points), 4)]
    sentinel_points = groups[0] if groups else ()
    remaining = groups[1:]
    z_min = min((group[0][2] for group in remaining), default=None)

    check_names = (
        "p0_x_matches_k_plus_0_001",
        "p1_xy_is_unit",
        "p1_z_is_zero",
        "p2_equals_p3",
        "p2_z_matches_z_min_plus_0_07",
    )
    checks = {name: False for name in check_names}
    decoded_k: Optional[int] = None
    p2_p3_error: Optional[float] = None
    expected_p2_z: Optional[float] = None
    if sentinel_points:
        p0, p1, p2, p3 = sentinel_points
        decoded_k = round(p0[0] - 0.001)
        checks["p0_x_matches_k_plus_0_001"] = _close(
            p0[0], decoded_k + 0.001, absolute_tolerance, relative_tolerance
        )
        checks["p1_xy_is_unit"] = _close(
            math.hypot(p1[0], p1[1]), 1.0, absolute_tolerance, relative_tolerance
        )
        checks["p1_z_is_zero"] = _close(
            p1[2], 0.0, absolute_tolerance, relative_tolerance
        )
        p2_p3_error = _point_error(p2, p3)
        checks["p2_equals_p3"] = _points_close(
            p2, p3, absolute_tolerance, relative_tolerance
        )
        if z_min is not None:
            expected_p2_z = z_min + 0.07
            checks["p2_z_matches_z_min_plus_0_07"] = _close(
                p2[2], expected_p2_z, absolute_tolerance, relative_tolerance
            )

    group_rows = []
    areas: list[float] = []
    winding = Counter()
    same_z_count = 0
    coplanar_count = 0
    plane_defined_count = 0
    zero_area_count = 0
    for relative_index, group in enumerate(remaining, start=1):
        area = _quadrangle_area(group)
        signed_xy_area = _signed_xy_area(group)
        if signed_xy_area > area_tolerance:
            winding_name = "counterclockwise"
        elif signed_xy_area < -area_tolerance:
            winding_name = "clockwise"
        else:
            winding_name = "degenerate_xy"
        z_values = [point[2] for point in group]
        z_range = max(z_values) - min(z_values)
        z_scale = max(1.0, *(abs(value) for value in z_values))
        same_z = z_range <= absolute_tolerance + relative_tolerance * z_scale
        coplanar, plane_defined, planarity_error = _planarity(
            group,
            absolute_tolerance,
            relative_tolerance,
            area_tolerance,
        )
        zero_area = area <= area_tolerance
        areas.append(area)
        winding[winding_name] += 1
        same_z_count += int(same_z)
        coplanar_count += int(coplanar)
        plane_defined_count += int(plane_defined)
        zero_area_count += int(zero_area)
        group_rows.append(
            {
                "group_index": relative_index,
                "winding_xy": winding_name,
                "signed_xy_area_m2": signed_xy_area,
                "area_3d_m2": area,
                "zero_area": zero_area,
                "same_z": same_z,
                "z_range_m": z_range,
                "coplanar": coplanar,
                "plane_defined": plane_defined,
                "maximum_planarity_error_m": planarity_error,
            }
        )

    pointwise_pairs = []
    cyclic_pairs = []
    reversed_pairs = []
    for left_offset, right_offset in itertools.combinations(range(len(remaining)), 2):
        left = remaining[left_offset]
        right = remaining[right_offset]
        left_index = left_offset + 1
        right_index = right_offset + 1
        pointwise, pointwise_error = _sequence_match(
            left, right, absolute_tolerance, relative_tolerance
        )
        if pointwise:
            pointwise_pairs.append(
                _pair_row(left_index, right_index, pointwise_error)
            )
            continue

        cyclic_match = False
        for shift in range(1, 4):
            rotated = tuple(right[(index + shift) % 4] for index in range(4))
            matches, error = _sequence_match(
                left, rotated, absolute_tolerance, relative_tolerance
            )
            if matches:
                cyclic_pairs.append(
                    _pair_row(
                        left_index,
                        right_index,
                        error,
                        right_cyclic_shift=shift,
                    )
                )
                cyclic_match = True
                break
        if cyclic_match:
            continue

        for shift in range(4):
            reversed_order = tuple(right[(shift - index) % 4] for index in range(4))
            matches, error = _sequence_match(
                left, reversed_order, absolute_tolerance, relative_tolerance
            )
            if matches:
                reversed_pairs.append(
                    _pair_row(
                        left_index,
                        right_index,
                        error,
                        right_reversed_start=shift,
                    )
                )
                break

    return {
        "sentinel": {
            "present": bool(sentinel_points),
            "points_xyz": [list(point) for point in sentinel_points],
            "decoded_k": decoded_k,
            "k_definition": "nearest integer to P0.x - 0.001; no business meaning inferred",
            "z_min_m": z_min,
            "z_min_definition": "minimum P0.z among non-sentinel factory groups",
            "expected_p2_z_m": expected_p2_z,
            "p2_p3_maximum_coordinate_error_m": p2_p3_error,
            "checks": checks,
            "all_checks_pass": all(checks.values()),
        },
        "remaining_group_count": len(remaining),
        "winding_xy": {
            "counterclockwise": winding["counterclockwise"],
            "clockwise": winding["clockwise"],
            "degenerate_xy": winding["degenerate_xy"],
        },
        "same_z_count": same_z_count,
        "coplanar_count": coplanar_count,
        "plane_defined_count": plane_defined_count,
        "area_3d_m2": _numeric_summary(areas, zero_area_count),
        "pointwise_duplicate_pairs": pointwise_pairs,
        "cyclic_vertex_duplicate_pairs": cyclic_pairs,
        "reversed_vertex_duplicate_pairs": reversed_pairs,
        "groups": group_rows,
    }


def _candidate_analysis(candidates: Sequence[Candidate], area_tolerance: float) -> dict[str, Any]:
    items = []
    areas = []
    empty_indices = []
    zero_indices = []
    union_indices = []
    for candidate_index, candidate in enumerate(candidates):
        area = _polygon_area(candidate.hull)
        empty_hull = len(candidate.hull) == 0
        zero_area = area <= area_tolerance
        if empty_hull:
            empty_indices.append(candidate_index)
        if zero_area:
            zero_indices.append(candidate_index)
        if empty_hull or zero_area:
            union_indices.append(candidate_index)
        areas.append(area)
        items.append(
            {
                "candidate_index": candidate_index,
                "type": candidate.type,
                "hull_point_count": len(candidate.hull),
                "hull_area_3d_m2": area,
                "empty_hull": empty_hull,
                "zero_area": zero_area,
            }
        )
    return {
        "count": len(candidates),
        "empty_hull_count": len(empty_indices),
        "zero_area_count": len(zero_indices),
        "zero_area_or_empty_hull_count": len(union_indices),
        "empty_hull_indices": empty_indices,
        "zero_area_indices": zero_indices,
        "zero_area_or_empty_hull_indices": union_indices,
        "hull_area_3d_m2": _numeric_summary(areas, len(zero_indices)),
        "items": items,
    }


def _format_key_set(keys: set[FrameKey]) -> str:
    return ", ".join(
        f"({case!r},{stamp_ns},{selected_index})"
        for case, stamp_ns, selected_index in sorted(keys)
    )


def build_analysis(
    replay_pack_bytes: bytes,
    replay_jsonl_bytes: bytes,
    area_tolerance: Optional[float] = None,
) -> dict[str, Any]:
    """Build the complete machine-readable offline parity report."""

    pack = parse_replay_pack(replay_pack_bytes)
    output_frames = parse_replay_jsonl(replay_jsonl_bytes)
    if area_tolerance is None:
        area_tolerance = pack.header.absolute_tolerance ** 2
    if not math.isfinite(area_tolerance) or area_tolerance < 0.0:
        raise ParityAnalysisError("area tolerance must be finite and non-negative")

    pack_by_key = {frame.key: frame for frame in pack.frames}
    output_by_key = {frame.key: frame for frame in output_frames}
    missing = set(pack_by_key) - set(output_by_key)
    extra = set(output_by_key) - set(pack_by_key)
    if missing or extra:
        details = []
        if missing:
            details.append("missing JSONL frames=" + _format_key_set(missing))
        if extra:
            details.append("extra JSONL frames=" + _format_key_set(extra))
        raise ParityAnalysisError("replay/JSONL frame sets differ: " + "; ".join(details))

    frame_rows = []
    all_factory_areas: list[float] = []
    all_candidate_areas: list[float] = []
    factory_winding = Counter()
    sentinel_checks = Counter()
    sentinel_all = 0
    remaining_group_count = 0
    same_z_count = 0
    coplanar_count = 0
    plane_defined_count = 0
    factory_zero_area_count = 0
    pointwise_pair_count = 0
    cyclic_pair_count = 0
    reversed_pair_count = 0
    factory_count_total = 0
    candidate_count_total = 0
    count_equal_frames = 0
    empty_hull_count = 0
    candidate_zero_area_count = 0
    candidate_union_count = 0

    for replay_frame in pack.frames:
        output_frame = output_by_key[replay_frame.key]
        expected_factory_count = len(replay_frame.factory_points) // 4
        if output_frame.factory_group_count != expected_factory_count:
            raise ParityAnalysisError(
                f"frame {replay_frame.key!r}: JSONL factory_group_count "
                f"{output_frame.factory_group_count} does not match replay pack "
                f"{expected_factory_count}"
            )
        if output_frame.retained != replay_frame.expected_retained_count:
            raise ParityAnalysisError(
                f"frame {replay_frame.key!r}: JSONL retained {output_frame.retained} "
                f"does not match replay pack {replay_frame.expected_retained_count}"
            )

        factory = _factory_analysis(
            replay_frame.factory_points,
            pack.header.absolute_tolerance,
            pack.header.relative_tolerance,
            area_tolerance,
        )
        candidates = _candidate_analysis(output_frame.candidates, area_tolerance)
        factory_count = expected_factory_count
        candidate_count = len(output_frame.candidates)
        frame_rows.append(
            {
                "case": replay_frame.case,
                "stamp_ns": replay_frame.stamp_ns,
                "selected_index": replay_frame.selected_index,
                "counts": {
                    "factory_groups": factory_count,
                    "candidates": candidate_count,
                    "candidate_minus_factory": candidate_count - factory_count,
                    "equal": candidate_count == factory_count,
                },
                "factory": factory,
                "candidates": candidates,
            }
        )

        factory_count_total += factory_count
        candidate_count_total += candidate_count
        count_equal_frames += int(factory_count == candidate_count)
        sentinel_all += int(factory["sentinel"]["all_checks_pass"])
        for name, passed in factory["sentinel"]["checks"].items():
            sentinel_checks[name] += int(passed)
        remaining_group_count += factory["remaining_group_count"]
        same_z_count += factory["same_z_count"]
        coplanar_count += factory["coplanar_count"]
        plane_defined_count += factory["plane_defined_count"]
        for name, count in factory["winding_xy"].items():
            factory_winding[name] += count
        all_factory_areas.extend(group["area_3d_m2"] for group in factory["groups"])
        factory_zero_area_count += factory["area_3d_m2"]["zero_count"]
        pointwise_pair_count += len(factory["pointwise_duplicate_pairs"])
        cyclic_pair_count += len(factory["cyclic_vertex_duplicate_pairs"])
        reversed_pair_count += len(factory["reversed_vertex_duplicate_pairs"])
        all_candidate_areas.extend(item["hull_area_3d_m2"] for item in candidates["items"])
        empty_hull_count += candidates["empty_hull_count"]
        candidate_zero_area_count += candidates["zero_area_count"]
        candidate_union_count += candidates["zero_area_or_empty_hull_count"]

    return {
        "schema": SCHEMA_NAME,
        "scope": {
            "kind": "offline_geometry_regression",
            "final_algorithm_recovery_claimed": False,
            "requires_network": False,
            "requires_robot": False,
        },
        "inputs": {
            "replay_pack": {
                "sha256": pack.sha256,
                "size_bytes": pack.size_bytes,
                "schema": "x30rpl/1.0",
                "source_manifest_sha256": pack.header.source_manifest_sha256,
                "body_sha256": pack.header.body_sha256,
            },
            "replay_jsonl": {
                "sha256": _sha256(replay_jsonl_bytes),
                "size_bytes": len(replay_jsonl_bytes),
            },
        },
        "tolerances": {
            "absolute_m": pack.header.absolute_tolerance,
            "relative": pack.header.relative_tolerance,
            "area_m2": area_tolerance,
        },
        "totals": {
            "frame_count": len(frame_rows),
            "counts": {
                "factory_groups": factory_count_total,
                "candidates": candidate_count_total,
                "frames_with_equal_factory_candidate_count": count_equal_frames,
            },
            "factory_sentinel": {
                "frames_with_all_checks_passed": sentinel_all,
                "check_passed_frames": {
                    name: sentinel_checks[name]
                    for name in sorted(sentinel_checks)
                },
            },
            "factory_remaining_groups": {
                "count": remaining_group_count,
                "winding_xy": {
                    "counterclockwise": factory_winding["counterclockwise"],
                    "clockwise": factory_winding["clockwise"],
                    "degenerate_xy": factory_winding["degenerate_xy"],
                },
                "same_z_count": same_z_count,
                "coplanar_count": coplanar_count,
                "plane_defined_count": plane_defined_count,
                "area_3d_m2": _numeric_summary(
                    all_factory_areas, factory_zero_area_count
                ),
                "pointwise_duplicate_pair_count": pointwise_pair_count,
                "cyclic_vertex_duplicate_pair_count": cyclic_pair_count,
                "reversed_vertex_duplicate_pair_count": reversed_pair_count,
            },
            "candidates": {
                "count": candidate_count_total,
                "empty_hull_count": empty_hull_count,
                "zero_area_count": candidate_zero_area_count,
                "zero_area_or_empty_hull_count": candidate_union_count,
                "hull_area_3d_m2": _numeric_summary(
                    all_candidate_areas, candidate_zero_area_count
                ),
            },
        },
        "frames": frame_rows,
    }


def render_analysis(analysis: Mapping[str, Any]) -> bytes:
    text = json.dumps(
        analysis,
        sort_keys=True,
        indent=2,
        ensure_ascii=True,
        allow_nan=False,
    )
    return (text + "\n").encode("ascii")


def analyze_files(
    replay_pack_path: Path,
    replay_jsonl_path: Path,
    output: str,
    area_tolerance: Optional[float] = None,
) -> dict[str, Any]:
    replay_pack_path = Path(replay_pack_path)
    replay_jsonl_path = Path(replay_jsonl_path)
    if output != "-":
        output_path = Path(output)
        resolved_output = output_path.resolve()
        if resolved_output in {
            replay_pack_path.resolve(),
            replay_jsonl_path.resolve(),
        }:
            raise ParityAnalysisError("output path must not overwrite either input")
    analysis = build_analysis(
        replay_pack_path.read_bytes(),
        replay_jsonl_path.read_bytes(),
        area_tolerance,
    )
    rendered = render_analysis(analysis)
    if output == "-":
        sys.stdout.buffer.write(rendered)
    else:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(rendered)
    return analysis


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replay-pack",
        "--source-pack",
        dest="replay_pack",
        required=True,
        type=Path,
        help="schema-v1 .x30rpl replay pack",
    )
    parser.add_argument(
        "--input-jsonl",
        "--input",
        dest="input_jsonl",
        required=True,
        type=Path,
        help="replay_fixture_test JSONL output",
    )
    parser.add_argument(
        "--output",
        default="-",
        help="output JSON path, or '-' for stdout (default)",
    )
    parser.add_argument(
        "--area-tolerance",
        type=float,
        help="zero-area threshold in square metres (default: replay abs tolerance squared)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        analyze_files(
            args.replay_pack,
            args.input_jsonl,
            args.output,
            args.area_tolerance,
        )
    except (OSError, ParityAnalysisError) as error:
        print(f"x30 quadrangle parity analysis failure: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
