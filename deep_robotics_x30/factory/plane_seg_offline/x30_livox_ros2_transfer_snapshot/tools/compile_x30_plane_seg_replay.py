#!/usr/bin/env python3
"""Compile paired plane-segmentation fixtures into a deterministic replay pack."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


MAGIC = b"X30RPLY\0"
SCHEMA_MAJOR = 1
SCHEMA_MINOR = 0
HEADER_BYTES = 128
HEADER_FLAGS = 3  # little-endian and IEEE-754
FRAME_RECORD_BYTES = 384
ABS_TOL = 1.0e-5
REL_TOL = 1.0e-5
UINT32_MAX = (1 << 32) - 1

HAS_EXACT_TF = 1
EXPECT_MISSING_TF = 2
HAS_FACTORY_ORACLE = 4
HAS_CORE_ORACLE = 8
RUN_CORE = 16

PAIRED_MANIFEST_SCHEMA = "x30-plane-seg-paired-manifest/v1"
PAIRED_FIXTURE_SCHEMA = "x30-plane-seg-paired-fixture/v1"
LAYER_ENCODING = (
    "IEEE-754 float32, little-endian, exact ROS serialized data bytes"
)

HEADER_STRUCT = struct.Struct("<8sHHIIIIIQQQ32s32sff")
FRAME_STRUCT = struct.Struct("<64sQ6I17d9fBBH4I8Q32s")
_FLOAT32 = struct.Struct("<f")
_POINT_XYZ = struct.Struct("<fff")
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")

assert HEADER_STRUCT.size == HEADER_BYTES
assert FRAME_STRUCT.size == FRAME_RECORD_BYTES


@dataclass(frozen=True)
class _Blob:
    path: str
    raw: bytes
    sha256: str


@dataclass(frozen=True)
class _Layer:
    name: str
    path: str
    raw: bytes
    size_x: int
    size_y: int


@dataclass(frozen=True)
class _ReplayFrame:
    source_bag_path: str
    case_name: str
    stamp_ns: int
    selected_index: int
    flags: int
    size_x: int
    size_y: int
    outer_start: int
    inner_start: int
    resolution: float
    length_x: float
    length_y: float
    center: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    world_to_base_translation: tuple[float, float, float]
    world_to_base_rotation: tuple[float, float, float, float]
    expected_retained_count: int
    factory_point_count: int
    elevation: bytes
    accessibility: bytes
    factory_xyz: bytes
    frame_input_sha256: bytes


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _sequence(value: Any, label: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be an array")
    return value


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _integer(value: Any, label: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    if value < 0 or value > maximum:
        raise ValueError(f"{label} is outside [0, {maximum}]")
    return value


def _u32(value: Any, label: str) -> int:
    return _integer(value, label, UINT32_MAX)


def _u64(value: Any, label: str) -> int:
    return _integer(value, label, (1 << 64) - 1)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    items = _sequence(value, label)
    if len(items) != length:
        raise ValueError(f"{label} must contain {length} values")
    return tuple(_finite(item, f"{label}[{index}]") for index, item in enumerate(items))


def _unit_quaternion(value: Any, label: str) -> tuple[float, float, float, float]:
    quaternion = _vector(value, 4, label)
    norm_squared = sum(component * component for component in quaternion)
    if not math.isclose(norm_squared, 1.0, rel_tol=1.0e-6, abs_tol=1.0e-6):
        raise ValueError(f"{label} is not a unit quaternion")
    return quaternion  # type: ignore[return-value]


def _sha256_hex(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{label} must be a 64-digit SHA-256 hex string")
    return value.lower()


def _normalize_frame_id(value: Any, label: str) -> str:
    frame_id = _text(value, label).lstrip("/")
    if not frame_id:
        raise ValueError(f"{label} must name a frame")
    return frame_id


def _require_world(value: Any, label: str) -> None:
    if _normalize_frame_id(value, label) != "world":
        raise ValueError(f"{label} must be world")


def _header_stamp(header_value: Any, label: str) -> int:
    header = _mapping(header_value, label)
    stamp_ns = _u64(header.get("stamp_ns"), f"{label}.stamp_ns")
    stamp_sec = _u32(header.get("stamp_sec"), f"{label}.stamp_sec")
    stamp_nsec = _u32(header.get("stamp_nsec"), f"{label}.stamp_nsec")
    if stamp_nsec >= 1_000_000_000:
        raise ValueError(f"{label}.stamp_nsec must be below one billion")
    if stamp_sec * 1_000_000_000 + stamp_nsec != stamp_ns:
        raise ValueError(f"{label} stamp fields disagree")
    return stamp_ns


def _member_path(root: Path, value: Any, label: str) -> tuple[str, Path]:
    relative = _text(value, label)
    if "\\" in relative:
        raise ValueError(f"{label} must use portable '/' separators")
    pure = PurePosixPath(relative)
    if pure.is_absolute() or not pure.parts or any(part in ("", ".", "..") for part in pure.parts):
        raise ValueError(f"{label} must be a normalized relative path")
    path = root.joinpath(*pure.parts)
    return pure.as_posix(), path


def _read_json(path: Path, label: str) -> tuple[bytes, Mapping[str, Any]]:
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise ValueError(f"cannot read {label}: {path}") from error
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is not valid UTF-8 JSON: {path}") from error
    return raw, _mapping(value, label)


def _read_manifest(source_dir: Path) -> tuple[bytes, Mapping[str, Any]]:
    manifest_path = source_dir / "manifest.json"
    manifest_bytes, manifest = _read_json(manifest_path, "manifest")
    sidecar_path = source_dir / "manifest.json.sha256"
    try:
        sidecar_lines = sidecar_path.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise ValueError(f"cannot read manifest SHA-256 sidecar: {sidecar_path}") from error
    if len(sidecar_lines) != 1:
        raise ValueError("manifest SHA-256 sidecar must contain exactly one line")
    fields = sidecar_lines[0].split()
    if len(fields) != 2 or fields[1] != "manifest.json":
        raise ValueError("manifest SHA-256 sidecar must name manifest.json")
    declared = _sha256_hex(fields[0], "manifest sidecar digest")
    actual = _sha256(manifest_bytes)
    if declared != actual:
        raise ValueError(f"manifest SHA-256 mismatch: declared={declared}, actual={actual}")
    if manifest.get("schema") != PAIRED_MANIFEST_SCHEMA:
        raise ValueError(f"unsupported paired manifest schema: {manifest.get('schema')!r}")
    hashing = _mapping(manifest.get("hashing"), "manifest.hashing")
    if hashing.get("algorithm") != "sha256":
        raise ValueError("manifest hashing algorithm must be sha256")
    return manifest_bytes, manifest


def _load_manifest_blobs(
    source_dir: Path,
    declarations: Any,
    label: str,
    globally_seen: set[str],
) -> dict[str, _Blob]:
    blobs: dict[str, _Blob] = {}
    for index, value in enumerate(_sequence(declarations, label)):
        item_label = f"{label}[{index}]"
        declaration = _mapping(value, item_label)
        relative, path = _member_path(source_dir, declaration.get("path"), f"{item_label}.path")
        if relative in globally_seen:
            raise ValueError(f"duplicate manifest blob path: {relative}")
        globally_seen.add(relative)
        expected_size = _u64(declaration.get("size_bytes"), f"{item_label}.size_bytes")
        expected_hash = _sha256_hex(declaration.get("sha256"), f"{item_label}.sha256")
        try:
            raw = path.read_bytes()
        except OSError as error:
            raise ValueError(f"cannot read declared blob: {path}") from error
        if len(raw) != expected_size:
            raise ValueError(
                f"blob size mismatch for {relative}: declared={expected_size}, actual={len(raw)}"
            )
        actual_hash = _sha256(raw)
        if actual_hash != expected_hash:
            raise ValueError(
                f"blob SHA-256 mismatch for {relative}: declared={expected_hash}, actual={actual_hash}"
            )
        blobs[relative] = _Blob(relative, raw, actual_hash)
    return blobs


def _validate_layer(
    source_dir: Path,
    value: Any,
    label: str,
    manifest_blobs: Mapping[str, _Blob],
) -> _Layer:
    layer = _mapping(value, label)
    name = _text(layer.get("name"), f"{label}.name")
    if layer.get("encoding") != LAYER_ENCODING:
        raise ValueError(f"{label}.encoding is not the required float32 encoding")
    dimensions = _sequence(layer.get("dimensions"), f"{label}.dimensions")
    if len(dimensions) != 2:
        raise ValueError(f"{label}.dimensions must contain exactly two dimensions")
    by_label: dict[str, tuple[int, int]] = {}
    for index, dimension_value in enumerate(dimensions):
        dimension_label = f"{label}.dimensions[{index}]"
        dimension = _mapping(dimension_value, dimension_label)
        axis = _text(dimension.get("label"), f"{dimension_label}.label")
        if axis in by_label:
            raise ValueError(f"{label} has duplicate dimension label {axis!r}")
        by_label[axis] = (
            _u32(dimension.get("size"), f"{dimension_label}.size"),
            _u32(dimension.get("stride"), f"{dimension_label}.stride"),
        )
    if set(by_label) != {"column_index", "row_index"}:
        raise ValueError(f"{label} dimensions must be column_index and row_index")
    size_x, column_stride = by_label["column_index"]
    size_y, row_stride = by_label["row_index"]
    if size_x == 0 or size_y == 0:
        raise ValueError(f"{label} dimensions must be non-zero")
    value_count = size_x * size_y
    if column_stride != value_count or row_stride != size_y:
        raise ValueError(
            f"{label} strides must be column={value_count}, row={size_y}"
        )
    if _u32(layer.get("data_offset"), f"{label}.data_offset") != 0:
        raise ValueError(f"{label}.data_offset must be zero")
    if _u64(layer.get("value_count"), f"{label}.value_count") != value_count:
        raise ValueError(f"{label}.value_count does not match its dimensions")
    expected_bytes = value_count * _FLOAT32.size
    if _u64(layer.get("blob_size_bytes"), f"{label}.blob_size_bytes") != expected_bytes:
        raise ValueError(f"{label}.blob_size_bytes does not match its dimensions")
    relative, _ = _member_path(source_dir, layer.get("blob_path"), f"{label}.blob_path")
    blob = manifest_blobs.get(relative)
    if blob is None:
        raise ValueError(f"{label}.blob_path is not declared by its manifest fixture: {relative}")
    layer_hash = _sha256_hex(layer.get("blob_sha256"), f"{label}.blob_sha256")
    if layer_hash != blob.sha256:
        raise ValueError(f"{label}.blob_sha256 disagrees with the manifest")
    if len(blob.raw) != expected_bytes:
        raise ValueError(f"{label} blob byte count does not match its dimensions")
    return _Layer(name, relative, blob.raw, size_x, size_y)


def _validate_geometry(
    value: Any,
    size_x: int,
    size_y: int,
    label: str,
) -> tuple[
    float,
    float,
    float,
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    geometry = _mapping(value, label)
    resolution = _finite(geometry.get("resolution"), f"{label}.resolution")
    length_x = _finite(geometry.get("length_x"), f"{label}.length_x")
    length_y = _finite(geometry.get("length_y"), f"{label}.length_y")
    if resolution <= 0.0 or length_x <= 0.0 or length_y <= 0.0:
        raise ValueError(f"{label} resolution and lengths must be positive")
    if not math.isclose(length_x, resolution * size_x, rel_tol=1.0e-9, abs_tol=1.0e-9):
        raise ValueError(f"{label}.length_x does not match resolution * size_x")
    if not math.isclose(length_y, resolution * size_y, rel_tol=1.0e-9, abs_tol=1.0e-9):
        raise ValueError(f"{label}.length_y does not match resolution * size_y")
    center = _vector(geometry.get("position_xyz"), 3, f"{label}.position_xyz")
    orientation = _unit_quaternion(
        geometry.get("orientation_xyzw"), f"{label}.orientation_xyzw"
    )
    return (
        resolution,
        length_x,
        length_y,
        center,  # type: ignore[return-value]
        orientation,
    )


def _transform_components(
    value: Any,
    expected_stamp: int,
    label: str,
) -> tuple[
    str,
    str,
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    transform = _mapping(value, label)
    header = _mapping(transform.get("header"), f"{label}.header")
    if _header_stamp(header, f"{label}.header") != expected_stamp:
        raise ValueError(f"{label} does not have the paired frame stamp")
    parent = _normalize_frame_id(header.get("frame_id"), f"{label}.header.frame_id")
    child = _normalize_frame_id(transform.get("child_frame_id"), f"{label}.child_frame_id")
    if "normalized_frame_id" in transform and transform["normalized_frame_id"] != parent:
        raise ValueError(f"{label}.normalized_frame_id disagrees with header.frame_id")
    if "normalized_child_frame_id" in transform and transform["normalized_child_frame_id"] != child:
        raise ValueError(f"{label}.normalized_child_frame_id disagrees with child_frame_id")
    translation = _vector(transform.get("translation_xyz"), 3, f"{label}.translation_xyz")
    rotation = _unit_quaternion(transform.get("rotation_xyzw"), f"{label}.rotation_xyzw")
    return (
        parent,
        child,
        translation,  # type: ignore[return-value]
        rotation,
    )


def _world_to_base(
    value: Any,
    stamp_ns: int,
    label: str,
) -> tuple[int, tuple[float, float, float], tuple[float, float, float, float]]:
    tf = _mapping(value, label)
    status = _text(tf.get("status"), f"{label}.status")
    required_count = _u32(
        tf.get("required_transform_match_count"),
        f"{label}.required_transform_match_count",
    )
    exact_values = _sequence(
        tf.get("exact_dynamic_transforms"), f"{label}.exact_dynamic_transforms"
    )
    required_matches: list[Mapping[str, Any]] = []
    for index, transform_value in enumerate(exact_values):
        transform_label = f"{label}.exact_dynamic_transforms[{index}]"
        transform = _mapping(transform_value, transform_label)
        parent, child, _, _ = _transform_components(
            transform, stamp_ns, transform_label
        )
        if parent == "world" and child == "base_link":
            required_matches.append(transform)
    if len(required_matches) != required_count:
        raise ValueError(f"{label}.required_transform_match_count is inconsistent")

    selected = tf.get("world_to_base_link")
    if selected is None:
        if status != "missing_exact_required_dynamic_tf" or required_count != 0:
            raise ValueError(f"{label} missing-TF metadata is inconsistent")
        return (
            EXPECT_MISSING_TF,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )

    if status != "exact_required_dynamic_tf" or required_count == 0:
        raise ValueError(f"{label} exact-TF metadata is inconsistent")
    selected_mapping = _mapping(selected, f"{label}.world_to_base_link")
    parent, child, translation, rotation = _transform_components(
        selected_mapping, stamp_ns, f"{label}.world_to_base_link"
    )
    if parent != "world" or child != "base_link":
        raise ValueError(f"{label}.world_to_base_link must be world -> base_link")
    if selected_mapping not in required_matches:
        raise ValueError(f"{label}.world_to_base_link is not an exact dynamic match")
    return HAS_EXACT_TF, translation, rotation


def _factory_xyz(value: Any, stamp_ns: int, label: str) -> tuple[bytes, int]:
    oracle = _mapping(value, label)
    header = _mapping(oracle.get("header"), f"{label}.header")
    if _header_stamp(header, f"{label}.header") != stamp_ns:
        raise ValueError(f"{label} stamp does not match GridMap stamp")
    _require_world(header.get("frame_id"), f"{label}.header.frame_id")
    points = _sequence(oracle.get("points_xyz"), f"{label}.points_xyz")
    point_count = len(points)
    if point_count == 0 or point_count % 4 != 0:
        raise ValueError(f"{label}.points_xyz must contain complete quadrangles")
    shape = _mapping(oracle.get("shape"), f"{label}.shape")
    declared_count = _u32(shape.get("point_count"), f"{label}.shape.point_count")
    width = _u32(shape.get("width"), f"{label}.shape.width")
    height = _u32(shape.get("height"), f"{label}.shape.height")
    if declared_count != point_count or width * height != point_count:
        raise ValueError(f"{label}.shape does not match points_xyz")
    output = bytearray()
    for index, point_value in enumerate(points):
        point = _vector(point_value, 3, f"{label}.points_xyz[{index}]")
        try:
            output.extend(_POINT_XYZ.pack(*point))
        except (OverflowError, struct.error) as error:
            raise ValueError(f"{label}.points_xyz[{index}] is outside float32 range") from error
    return bytes(output), point_count


def _retained_count(elevation: bytes, accessibility: bytes) -> int:
    elevation_values = struct.iter_unpack("<f", elevation)
    accessibility_values = struct.iter_unpack("<f", accessibility)
    return sum(
        math.isfinite(elevation_value[0])
        and math.isfinite(accessibility_value[0])
        and accessibility_value[0] <= 0.9
        for elevation_value, accessibility_value in zip(
            elevation_values, accessibility_values
        )
    )


def _case_name(source_bag_path: str, label: str) -> str:
    name = PurePosixPath(source_bag_path).stem
    if not name or "\0" in name:
        raise ValueError(f"{label} does not produce a valid case name")
    try:
        encoded = name.encode("utf-8")
    except UnicodeEncodeError as error:
        raise ValueError(f"{label} case name is not valid UTF-8") from error
    if len(encoded) > 63:
        raise ValueError(f"{label} case name exceeds 63 UTF-8 bytes")
    return name


def _load_frame(
    source_dir: Path,
    source_bag_path: str,
    value: Any,
    label: str,
    manifest_blobs: Mapping[str, _Blob],
) -> tuple[_ReplayFrame, set[str]]:
    frame = _mapping(value, label)
    selected_index = _u32(frame.get("selected_frame_index"), f"{label}.selected_frame_index")
    grid_map = _mapping(frame.get("grid_map"), f"{label}.grid_map")
    grid_header = _mapping(grid_map.get("header"), f"{label}.grid_map.header")
    stamp_ns = _header_stamp(grid_header, f"{label}.grid_map.header")
    _require_world(grid_header.get("frame_id"), f"{label}.grid_map.header.frame_id")

    layer_values = _sequence(grid_map.get("layers"), f"{label}.grid_map.layers")
    layers: dict[str, _Layer] = {}
    referenced_paths: set[str] = set()
    layout: tuple[int, int] | None = None
    for index, layer_value in enumerate(layer_values):
        layer = _validate_layer(
            source_dir,
            layer_value,
            f"{label}.grid_map.layers[{index}]",
            manifest_blobs,
        )
        if layer.name in layers:
            raise ValueError(f"{label} has duplicate GridMap layer {layer.name!r}")
        if layer.path in referenced_paths:
            raise ValueError(f"{label} reuses GridMap blob {layer.path}")
        layers[layer.name] = layer
        referenced_paths.add(layer.path)
        current_layout = (layer.size_x, layer.size_y)
        if layout is None:
            layout = current_layout
        elif layout != current_layout:
            raise ValueError(f"{label} GridMap layers have inconsistent dimensions")
    if "elevation" not in layers or "accessibility" not in layers:
        raise ValueError(f"{label} must contain elevation and accessibility layers")
    basic_layers = _sequence(grid_map.get("basic_layers"), f"{label}.grid_map.basic_layers")
    if "elevation" not in basic_layers:
        raise ValueError(f"{label}.grid_map.basic_layers must contain elevation")
    elevation = layers["elevation"]
    accessibility = layers["accessibility"]
    size_x, size_y = elevation.size_x, elevation.size_y
    resolution, length_x, length_y, center, orientation = _validate_geometry(
        grid_map.get("geometry"), size_x, size_y, f"{label}.grid_map.geometry"
    )
    outer_start = _u32(grid_map.get("outer_start_index"), f"{label}.grid_map.outer_start_index")
    inner_start = _u32(grid_map.get("inner_start_index"), f"{label}.grid_map.inner_start_index")
    if outer_start >= size_x or inner_start >= size_y:
        raise ValueError(f"{label} GridMap start index is outside its dimensions")

    quadrangles = frame.get("expected_quadrangles")
    expected_quadrangles_hash = _sha256_hex(
        frame.get("expected_quadrangles_sha256"),
        f"{label}.expected_quadrangles_sha256",
    )
    actual_quadrangles_hash = _sha256(_canonical_json_bytes(quadrangles))
    if actual_quadrangles_hash != expected_quadrangles_hash:
        raise ValueError(f"{label} expected quadrangles SHA-256 mismatch")
    factory_xyz, factory_point_count = _factory_xyz(
        quadrangles, stamp_ns, f"{label}.expected_quadrangles"
    )
    tf_flag, translation, rotation = _world_to_base(
        frame.get("tf"), stamp_ns, f"{label}.tf"
    )
    flags = RUN_CORE | HAS_FACTORY_ORACLE | tf_flag
    frame_input_hash = hashlib.sha256(
        elevation.raw + accessibility.raw + factory_xyz
    ).digest()
    return (
        _ReplayFrame(
            source_bag_path=source_bag_path,
            case_name=_case_name(source_bag_path, f"{label}.source_bag_path"),
            stamp_ns=stamp_ns,
            selected_index=selected_index,
            flags=flags,
            size_x=size_x,
            size_y=size_y,
            outer_start=outer_start,
            inner_start=inner_start,
            resolution=resolution,
            length_x=length_x,
            length_y=length_y,
            center=center,
            orientation=orientation,
            world_to_base_translation=translation,
            world_to_base_rotation=rotation,
            expected_retained_count=_retained_count(
                elevation.raw, accessibility.raw
            ),
            factory_point_count=factory_point_count,
            elevation=elevation.raw,
            accessibility=accessibility.raw,
            factory_xyz=factory_xyz,
            frame_input_sha256=frame_input_hash,
        ),
        referenced_paths,
    )


def _load_frames(source_dir: Path, manifest: Mapping[str, Any]) -> list[_ReplayFrame]:
    fixture_entries = _sequence(manifest.get("fixtures"), "manifest.fixtures")
    if not fixture_entries:
        raise ValueError("manifest.fixtures must not be empty")
    frames: list[_ReplayFrame] = []
    seen_fixture_paths: set[str] = set()
    seen_blob_paths: set[str] = set()
    seen_source_bags: set[str] = set()
    seen_case_names: set[str] = set()

    for entry_index, entry_value in enumerate(fixture_entries):
        entry_label = f"manifest.fixtures[{entry_index}]"
        entry = _mapping(entry_value, entry_label)
        fixture_relative, fixture_path = _member_path(
            source_dir, entry.get("fixture_path"), f"{entry_label}.fixture_path"
        )
        if fixture_relative in seen_fixture_paths:
            raise ValueError(f"duplicate fixture path: {fixture_relative}")
        seen_fixture_paths.add(fixture_relative)
        fixture_bytes, fixture = _read_json(fixture_path, f"fixture {fixture_relative}")
        expected_fixture_size = _u64(
            entry.get("fixture_size_bytes"), f"{entry_label}.fixture_size_bytes"
        )
        if len(fixture_bytes) != expected_fixture_size:
            raise ValueError(f"fixture size mismatch for {fixture_relative}")
        expected_fixture_hash = _sha256_hex(
            entry.get("fixture_sha256"), f"{entry_label}.fixture_sha256"
        )
        actual_fixture_hash = _sha256(fixture_bytes)
        if actual_fixture_hash != expected_fixture_hash:
            raise ValueError(f"fixture SHA-256 mismatch for {fixture_relative}")
        if fixture.get("schema") != PAIRED_FIXTURE_SCHEMA:
            raise ValueError(f"unsupported paired fixture schema in {fixture_relative}")
        hashing = _mapping(fixture.get("hashing"), f"fixture {fixture_relative}.hashing")
        if hashing.get("algorithm") != "sha256":
            raise ValueError(f"fixture {fixture_relative} hashing algorithm must be sha256")

        source = _mapping(fixture.get("source"), f"fixture {fixture_relative}.source")
        source_bag_path = _text(
            source.get("bag_path"), f"fixture {fixture_relative}.source.bag_path"
        )
        if source_bag_path != entry.get("source_bag_path"):
            raise ValueError(f"source bag path mismatch for {fixture_relative}")
        if source_bag_path in seen_source_bags:
            raise ValueError(f"duplicate source bag path: {source_bag_path}")
        seen_source_bags.add(source_bag_path)
        source_bag_hash = _sha256_hex(
            source.get("bag_sha256"), f"fixture {fixture_relative}.source.bag_sha256"
        )
        entry_bag_hash = _sha256_hex(
            entry.get("source_bag_sha256"), f"{entry_label}.source_bag_sha256"
        )
        if source_bag_hash != entry_bag_hash:
            raise ValueError(f"source bag SHA-256 declaration mismatch for {fixture_relative}")

        case_name = _case_name(source_bag_path, f"fixture {fixture_relative}.source.bag_path")
        if case_name in seen_case_names:
            raise ValueError(f"duplicate replay case name: {case_name}")
        seen_case_names.add(case_name)
        manifest_blobs = _load_manifest_blobs(
            source_dir,
            entry.get("blobs"),
            f"{entry_label}.blobs",
            seen_blob_paths,
        )
        fixture_frames = _sequence(fixture.get("frames"), f"fixture {fixture_relative}.frames")
        expected_frame_count = _u32(
            entry.get("selected_frame_count"), f"{entry_label}.selected_frame_count"
        )
        if len(fixture_frames) != expected_frame_count:
            raise ValueError(f"selected frame count mismatch for {fixture_relative}")
        selected_indices: set[int] = set()
        referenced_paths: set[str] = set()
        for frame_index, frame_value in enumerate(fixture_frames):
            replay_frame, frame_paths = _load_frame(
                source_dir,
                source_bag_path,
                frame_value,
                f"fixture {fixture_relative}.frames[{frame_index}]",
                manifest_blobs,
            )
            if replay_frame.selected_index in selected_indices:
                raise ValueError(
                    f"duplicate selected frame index {replay_frame.selected_index} in {fixture_relative}"
                )
            selected_indices.add(replay_frame.selected_index)
            if referenced_paths & frame_paths:
                raise ValueError(f"fixture {fixture_relative} reuses a layer blob across frames")
            referenced_paths.update(frame_paths)
            frames.append(replay_frame)
        if referenced_paths != set(manifest_blobs):
            missing = sorted(set(manifest_blobs) - referenced_paths)
            extra = sorted(referenced_paths - set(manifest_blobs))
            raise ValueError(
                f"fixture {fixture_relative} blob declarations do not match layers: "
                f"unreferenced={missing}, undeclared={extra}"
            )

    frames.sort(key=lambda frame: (frame.source_bag_path, frame.selected_index))
    if len(frames) > UINT32_MAX:
        raise ValueError("replay frame count exceeds uint32")
    return frames


def _case_name_bytes(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return encoded + b"\0" * (64 - len(encoded))


def _pack_frame(
    frame: _ReplayFrame,
    elevation_offset: int,
    accessibility_offset: int,
    factory_offset: int,
) -> bytes:
    return FRAME_STRUCT.pack(
        _case_name_bytes(frame.case_name),
        frame.stamp_ns,
        frame.selected_index,
        frame.flags,
        frame.size_x,
        frame.size_y,
        frame.outer_start,
        frame.inner_start,
        frame.resolution,
        frame.length_x,
        frame.length_y,
        *frame.center,
        *frame.orientation,
        *frame.world_to_base_translation,
        *frame.world_to_base_rotation,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.9,
        0.01,
        45.0,
        0,
        0,
        0,
        frame.expected_retained_count,
        UINT32_MAX,
        frame.factory_point_count,
        0,
        elevation_offset,
        len(frame.elevation),
        accessibility_offset,
        len(frame.accessibility),
        factory_offset,
        len(frame.factory_xyz),
        0,
        0,
        frame.frame_input_sha256,
    )


def compile_replay(source_dir: Path) -> bytes:
    """Validate a paired fixture corpus and return its deterministic replay bytes."""

    source_dir = Path(source_dir)
    manifest_bytes, manifest = _read_manifest(source_dir)
    frames = _load_frames(source_dir, manifest)
    payload_offset = HEADER_BYTES + len(frames) * FRAME_RECORD_BYTES
    payload_cursor = payload_offset
    records = bytearray()
    payload = bytearray()
    for frame in frames:
        elevation_offset = payload_cursor
        payload.extend(frame.elevation)
        payload_cursor += len(frame.elevation)
        accessibility_offset = payload_cursor
        payload.extend(frame.accessibility)
        payload_cursor += len(frame.accessibility)
        factory_offset = payload_cursor
        payload.extend(frame.factory_xyz)
        payload_cursor += len(frame.factory_xyz)
        records.extend(
            _pack_frame(
                frame,
                elevation_offset,
                accessibility_offset,
                factory_offset,
            )
        )
    body = bytes(records + payload)
    file_bytes = HEADER_BYTES + len(body)
    header = HEADER_STRUCT.pack(
        MAGIC,
        SCHEMA_MAJOR,
        SCHEMA_MINOR,
        HEADER_BYTES,
        HEADER_FLAGS,
        len(frames),
        FRAME_RECORD_BYTES,
        0,
        HEADER_BYTES,
        payload_offset,
        file_bytes,
        hashlib.sha256(manifest_bytes).digest(),
        hashlib.sha256(body).digest(),
        ABS_TOL,
        REL_TOL,
    )
    return header + body


def default_sidecar_path(output_path: Path) -> Path:
    return Path(f"{output_path}.sha256")


def write_replay(source_dir: Path, output_path: Path) -> tuple[str, int]:
    """Compile a replay pack and write it with an appended .sha256 sidecar."""

    output_path = Path(output_path)
    replay = compile_replay(source_dir)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(replay)
    digest = _sha256(replay)
    default_sidecar_path(output_path).write_text(
        f"{digest}  {output_path.name}\n",
        encoding="ascii",
        newline="\n",
    )
    return digest, len(replay)


def _default_source_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "tests" / "fixtures" / "plane_seg_paired"


def _default_output_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "ws"
        / "src"
        / "x30_plane_seg_core"
        / "test"
        / "fixtures"
        / "x30_plane_seg_replay_v1.x30rpl"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        "--source-dir",
        dest="source_dir",
        type=Path,
        default=_default_source_dir(),
        help="paired fixture directory containing manifest.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=_default_output_path(),
        help="output .x30rpl path",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    digest, size = write_replay(args.source_dir, args.output)
    frame_count = HEADER_STRUCT.unpack_from(args.output.read_bytes())[5]
    print(
        f"wrote {frame_count} frames and {size} bytes to {args.output} "
        f"(sha256 {digest})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
