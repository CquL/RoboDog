#!/usr/bin/env python3
"""Export deterministic GridMap, quadrangle, and exact-TF offline fixtures."""

from __future__ import annotations

import argparse
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from analyze_x30_gridmap_baseline import (
    GridMapFrame,
    PointCloudFrame,
    RosbagPlaneSegInputs,
    TFMessageFrame,
    TransformStampedFrame,
    normalize_frame_id,
    read_rosbag_plane_seg_inputs,
)
from export_x30_plane_seg_fixtures import (
    CANONICAL_JSON_DESCRIPTION,
    canonical_json_bytes,
    default_source_root,
    formatted_json_bytes,
    relative_source_path,
    sha256_bytes,
    sha256_file,
    sidecar_metadata,
)


SCHEMA = "x30-plane-seg-paired-fixture/v1"
MANIFEST_SCHEMA = "x30-plane-seg-paired-manifest/v1"
EXPORTER_PATH = (
    "x30_livox_ros2_transfer/tools/"
    "export_x30_plane_seg_paired_fixtures.py"
)
PARSER_PATH = "x30_livox_ros2_transfer/tools/analyze_x30_gridmap_baseline.py"
DEFAULT_GRID_MAP_TOPIC = "/deeprobotics_local_height_map_mid360/height_map"
DEFAULT_QUADRANGLES_TOPIC = "/plane_seg/quadrangels"
DEFAULT_LOOK_POSE_TOPIC = "/plane_seg/look_pose"
DEFAULT_HEIGHT_MAP_MODE_TOPIC = "/height_map_mode"
DEFAULT_HEIGHT_MAP_MODE_STATE_TOPIC = "/height_map_mode_state"
DEFAULT_TF_TOPIC = "/tf"
DEFAULT_TF_STATIC_TOPIC = "/tf_static"
DEFAULT_PARENT_FRAME = "world"
DEFAULT_CHILD_FRAME = "base_link"
LAYER_ENCODING = "IEEE-754 float32, little-endian, exact ROS serialized data bytes"


@dataclass(frozen=True)
class RecordedTransform:
    topic: str
    record_time_ns: int
    message_index: int
    transform_index: int
    transform: TransformStampedFrame


@dataclass(frozen=True)
class PairedFrame:
    pair_index: int
    grid_map_index: int
    quadrangles_index: int
    grid_map: GridMapFrame
    quadrangles: PointCloudFrame
    exact_dynamic_transforms: tuple[RecordedTransform, ...]
    required_transform_matches: tuple[RecordedTransform, ...]

    @property
    def stamp_ns(self) -> int:
        return self.grid_map.stamp_ns

    @property
    def has_required_tf(self) -> bool:
        return bool(self.required_transform_matches)


@dataclass(frozen=True)
class PairingResult:
    exact_pairs: tuple[PairedFrame, ...]
    pairs_with_required_tf: tuple[PairedFrame, ...]
    missing_required_tf: tuple[PairedFrame, ...]
    orphan_grid_maps: tuple[tuple[int, GridMapFrame], ...]
    orphan_quadrangles: tuple[tuple[int, PointCloudFrame], ...]
    static_transforms: tuple[RecordedTransform, ...]

    @property
    def complete_pairs(self) -> tuple[PairedFrame, ...]:
        """GridMap/quadrangles pairs complete at an exact Header.stamp."""
        return self.exact_pairs


def transform_sort_key(recorded: RecordedTransform) -> tuple[Any, ...]:
    transform = recorded.transform
    return (
        transform.stamp_ns,
        transform.normalized_frame_id,
        transform.normalized_child_frame_id,
        transform.frame_id,
        transform.child_frame_id,
        transform.translation,
        transform.rotation,
        recorded.record_time_ns,
        recorded.message_index,
        recorded.transform_index,
    )


def flatten_transforms(
    messages: Sequence[TFMessageFrame],
) -> tuple[RecordedTransform, ...]:
    transforms = [
        RecordedTransform(
            topic=message.topic,
            record_time_ns=message.record_time_ns,
            message_index=message_index,
            transform_index=transform_index,
            transform=transform,
        )
        for message_index, message in enumerate(messages)
        for transform_index, transform in enumerate(message.transforms)
    ]
    return tuple(sorted(transforms, key=transform_sort_key))


def unique_stamp_index(
    frames: Sequence[GridMapFrame] | Sequence[PointCloudFrame],
    label: str,
) -> dict[int, tuple[int, GridMapFrame | PointCloudFrame]]:
    indexed: dict[int, tuple[int, GridMapFrame | PointCloudFrame]] = {}
    for index, frame in enumerate(frames):
        if frame.stamp_ns in indexed:
            raise ValueError(
                f"duplicate {label} Header.stamp {frame.stamp_ns} is ambiguous"
            )
        indexed[frame.stamp_ns] = (index, frame)
    return indexed


def pair_plane_seg_inputs(
    inputs: RosbagPlaneSegInputs,
    parent_frame: str = DEFAULT_PARENT_FRAME,
    child_frame: str = DEFAULT_CHILD_FRAME,
) -> PairingResult:
    required_parent = normalize_frame_id(parent_frame)
    required_child = normalize_frame_id(child_frame)
    if not required_parent or not required_child:
        raise ValueError("required TF parent and child frame ids must be non-empty")

    grid_maps = unique_stamp_index(inputs.grid_maps, "GridMap")
    quadrangles = unique_stamp_index(inputs.quadrangles, "quadrangles")
    dynamic_transforms = flatten_transforms(inputs.tf_messages)
    static_transforms = flatten_transforms(inputs.tf_static_messages)
    dynamic_by_stamp: dict[int, list[RecordedTransform]] = defaultdict(list)
    for recorded in dynamic_transforms:
        dynamic_by_stamp[recorded.transform.stamp_ns].append(recorded)

    shared_stamps = sorted(grid_maps.keys() & quadrangles.keys())
    exact_pairs = []
    for pair_index, stamp_ns in enumerate(shared_stamps):
        grid_map_index, grid_map = grid_maps[stamp_ns]
        quadrangles_index, quadrangle = quadrangles[stamp_ns]
        exact_dynamic = tuple(dynamic_by_stamp.get(stamp_ns, ()))
        required_matches = tuple(
            recorded
            for recorded in exact_dynamic
            if recorded.transform.normalized_frame_id == required_parent
            and recorded.transform.normalized_child_frame_id == required_child
        )
        exact_pairs.append(
            PairedFrame(
                pair_index=pair_index,
                grid_map_index=grid_map_index,
                quadrangles_index=quadrangles_index,
                grid_map=grid_map,
                quadrangles=quadrangle,
                exact_dynamic_transforms=exact_dynamic,
                required_transform_matches=required_matches,
            )
        )

    pairs_with_required_tf = tuple(
        pair for pair in exact_pairs if pair.has_required_tf
    )
    missing_required_tf = tuple(
        pair for pair in exact_pairs if not pair.has_required_tf
    )
    orphan_grid_maps = tuple(
        (index, frame)
        for stamp_ns, (index, frame) in sorted(grid_maps.items())
        if stamp_ns not in quadrangles
    )
    orphan_quadrangles = tuple(
        (index, frame)
        for stamp_ns, (index, frame) in sorted(quadrangles.items())
        if stamp_ns not in grid_maps
    )
    return PairingResult(
        exact_pairs=tuple(exact_pairs),
        pairs_with_required_tf=pairs_with_required_tf,
        missing_required_tf=missing_required_tf,
        orphan_grid_maps=orphan_grid_maps,
        orphan_quadrangles=orphan_quadrangles,
        static_transforms=static_transforms,
    )


def parse_frames_per_bag(value: int | str) -> int:
    if isinstance(value, str):
        if value.lower() == "all":
            return 0
        try:
            value = int(value)
        except ValueError as error:
            raise ValueError(
                "frames per bag must be a non-negative integer or 'all'"
            ) from error
    if value < 0:
        raise ValueError("frames per bag must be non-negative")
    return value


def select_frame_indices(frame_count: int, requested_count: int | str) -> list[int]:
    if frame_count < 1:
        raise ValueError("cannot select fixtures without a complete paired frame")
    requested = parse_frames_per_bag(requested_count)
    if requested == 0 or requested >= frame_count:
        return list(range(frame_count))
    if requested == 1:
        return [frame_count // 2]
    return [
        index * (frame_count - 1) // (requested - 1)
        for index in range(requested)
    ]


def transform_metadata(recorded: RecordedTransform) -> dict[str, Any]:
    transform = recorded.transform
    return {
        "child_frame_id": transform.child_frame_id,
        "header": {
            "frame_id": transform.frame_id,
            "sequence": transform.sequence,
            "stamp_nsec": transform.stamp_nsec,
            "stamp_ns": transform.stamp_ns,
            "stamp_sec": transform.stamp_sec,
        },
        "normalized_child_frame_id": transform.normalized_child_frame_id,
        "normalized_frame_id": transform.normalized_frame_id,
        "record_time_ns": recorded.record_time_ns,
        "rotation_xyzw": list(transform.rotation),
        "source_message_index": recorded.message_index,
        "source_transform_index": recorded.transform_index,
        "topic": recorded.topic,
        "translation_xyz": list(transform.translation),
    }


def frame_reference(
    index: int,
    frame: GridMapFrame | PointCloudFrame,
) -> dict[str, Any]:
    return {
        "frame_id": frame.frame_id,
        "record_time_ns": frame.record_time_ns,
        "source_frame_index": index,
        "stamp_nsec": frame.stamp_nsec,
        "stamp_ns": frame.stamp_ns,
        "stamp_sec": frame.stamp_sec,
    }


def incomplete_pair_metadata(pair: PairedFrame) -> dict[str, Any]:
    return {
        "exact_dynamic_tf_edges": [
            {
                "child_frame_id": recorded.transform.child_frame_id,
                "frame_id": recorded.transform.frame_id,
                "normalized_child_frame_id": (
                    recorded.transform.normalized_child_frame_id
                ),
                "normalized_frame_id": recorded.transform.normalized_frame_id,
            }
            for recorded in pair.exact_dynamic_transforms
        ],
        "grid_map_source_frame_index": pair.grid_map_index,
        "pair_index": pair.pair_index,
        "quadrangles_source_frame_index": pair.quadrangles_index,
        "stamp_ns": pair.stamp_ns,
        "status": "missing_exact_required_dynamic_tf",
    }


def counter_metadata(values: Counter[int]) -> dict[str, int]:
    return {str(key): values[key] for key in sorted(values)}


def pairing_metadata(
    inputs: RosbagPlaneSegInputs,
    pairing: PairingResult,
    parent_frame: str,
    child_frame: str,
) -> dict[str, Any]:
    return {
        "counts": {
            "complete_pairs": len(pairing.complete_pairs),
            "dynamic_tf_messages": len(inputs.tf_messages),
            "exact_grid_map_quadrangles_pairs": len(pairing.exact_pairs),
            "grid_maps": len(inputs.grid_maps),
            "look_poses": len(inputs.look_poses),
            "missing_required_tf": len(pairing.missing_required_tf),
            "orphan_grid_maps": len(pairing.orphan_grid_maps),
            "orphan_quadrangles": len(pairing.orphan_quadrangles),
            "pairs_with_required_tf": len(pairing.pairs_with_required_tf),
            "quadrangles": len(inputs.quadrangles),
            "static_tf_messages": len(inputs.tf_static_messages),
            "static_transforms": len(pairing.static_transforms),
        },
        "look_pose": {
            "header_stamp_zero_count": sum(
                pose.stamp_ns == 0 for pose in inputs.look_poses
            ),
            "used_as_pairing_key": False,
        },
        "missing_required_tf": [
            incomplete_pair_metadata(pair) for pair in pairing.missing_required_tf
        ],
        "orphan_grid_maps": [
            frame_reference(index, frame)
            for index, frame in pairing.orphan_grid_maps
        ],
        "orphan_quadrangles": [
            frame_reference(index, frame)
            for index, frame in pairing.orphan_quadrangles
        ],
        "policy": {
            "dynamic_tf": "exact TransformStamped Header.stamp only",
            "grid_map_quadrangles": "exact Header.stamp only",
            "nearest_or_future_tf_fallback": False,
            "required_child_frame": normalize_frame_id(child_frame),
            "required_parent_frame": normalize_frame_id(parent_frame),
            "static_tf": "valid for the entire source bag",
        },
        "topic_values": {
            DEFAULT_HEIGHT_MAP_MODE_TOPIC: {
                "message_count": sum(inputs.height_map_mode_values.values()),
                "value_counts": counter_metadata(inputs.height_map_mode_values),
            },
            DEFAULT_HEIGHT_MAP_MODE_STATE_TOPIC: {
                "message_count": sum(
                    inputs.height_map_mode_state_values.values()
                ),
                "value_counts": counter_metadata(
                    inputs.height_map_mode_state_values
                ),
            },
        },
    }


def safe_layer_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    return sanitized or "unnamed"


def write_layer_blobs(
    output_dir: Path,
    blob_relative_dir: Path,
    frame: GridMapFrame,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    layer_metadata = []
    blob_entries = []
    for layer_index, layer in enumerate(frame.layers):
        expected_size = len(layer.values) * 4
        if len(layer.raw_data) != expected_size:
            raise ValueError(
                f"GridMap layer {layer.name!r} has {len(layer.raw_data)} raw bytes "
                f"for {len(layer.values)} float32 values"
            )
        blob_name = (
            f"layer_{layer_index:02d}_{safe_layer_name(layer.name)}.f32le"
        )
        blob_relative_path = blob_relative_dir / blob_name
        blob_path = output_dir / blob_relative_path
        blob_path.parent.mkdir(parents=True, exist_ok=True)
        blob_path.write_bytes(layer.raw_data)
        blob_hash = sha256_bytes(layer.raw_data)
        blob_path_text = blob_relative_path.as_posix()
        layer_metadata.append(
            {
                "blob_path": blob_path_text,
                "blob_sha256": blob_hash,
                "blob_size_bytes": len(layer.raw_data),
                "data_offset": layer.data_offset,
                "dimensions": [
                    {
                        "label": dimension.label,
                        "size": dimension.size,
                        "stride": dimension.stride,
                    }
                    for dimension in layer.dimensions
                ],
                "encoding": LAYER_ENCODING,
                "name": layer.name,
                "value_count": len(layer.values),
            }
        )
        blob_entries.append(
            {
                "path": blob_path_text,
                "sha256": blob_hash,
                "size_bytes": len(layer.raw_data),
            }
        )
    return layer_metadata, blob_entries


def quadrangles_metadata(frame: PointCloudFrame) -> dict[str, Any]:
    return {
        "header": {
            "frame_id": frame.frame_id,
            "stamp_nsec": frame.stamp_nsec,
            "stamp_ns": frame.stamp_ns,
            "stamp_sec": frame.stamp_sec,
        },
        "points_xyz": [list(point) for point in frame.points],
        "record_time_ns": frame.record_time_ns,
        "shape": {
            "height": frame.height,
            "point_count": len(frame.points),
            "width": frame.width,
        },
    }


def export_frame(
    output_dir: Path,
    blob_root: Path,
    selected_index: int,
    pair: PairedFrame,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    frame_blob_dir = blob_root / "frames" / (
        f"{selected_index:06d}_{pair.stamp_ns}"
    )
    layers, blobs = write_layer_blobs(
        output_dir=output_dir,
        blob_relative_dir=frame_blob_dir,
        frame=pair.grid_map,
    )
    expected_quadrangles = quadrangles_metadata(pair.quadrangles)
    expected_bytes = canonical_json_bytes(expected_quadrangles)
    required = (
        pair.required_transform_matches[0]
        if pair.required_transform_matches
        else None
    )
    return (
        {
            "expected_quadrangles": expected_quadrangles,
            "expected_quadrangles_sha256": sha256_bytes(expected_bytes),
            "grid_map": {
                "basic_layers": list(pair.grid_map.basic_layers),
                "geometry": {
                    "length_x": pair.grid_map.length_x,
                    "length_y": pair.grid_map.length_y,
                    "orientation_xyzw": list(pair.grid_map.orientation),
                    "position_xyz": list(pair.grid_map.position),
                    "resolution": pair.grid_map.resolution,
                },
                "header": {
                    "frame_id": pair.grid_map.frame_id,
                    "sequence": pair.grid_map.sequence,
                    "stamp_nsec": pair.grid_map.stamp_nsec,
                    "stamp_ns": pair.grid_map.stamp_ns,
                    "stamp_sec": pair.grid_map.stamp_sec,
                },
                "inner_start_index": pair.grid_map.inner_start_index,
                "layers": layers,
                "outer_start_index": pair.grid_map.outer_start_index,
                "record_time_ns": pair.grid_map.record_time_ns,
            },
            "pair_index": pair.pair_index,
            "selected_frame_index": selected_index,
            "source_frame_indices": {
                "grid_map": pair.grid_map_index,
                "quadrangles": pair.quadrangles_index,
            },
            "tf": {
                "exact_dynamic_transforms": [
                    transform_metadata(recorded)
                    for recorded in pair.exact_dynamic_transforms
                ],
                "required_transform_match_count": len(
                    pair.required_transform_matches
                ),
                "status": (
                    "exact_required_dynamic_tf"
                    if required is not None
                    else "missing_exact_required_dynamic_tf"
                ),
                "world_to_base_link": (
                    transform_metadata(required) if required is not None else None
                ),
            },
        },
        blobs,
    )


def build_source_metadata(
    bag_path: Path,
    source_root: Path,
    bag_sha256: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    sidecars, capture_metadata, declared_sha256, declared_path = sidecar_metadata(
        bag_path, source_root
    )
    if declared_sha256 is not None and declared_sha256 != bag_sha256:
        raise ValueError(
            f"SHA-256 mismatch for {bag_path}: "
            f"sidecar={declared_sha256}, actual={bag_sha256}"
        )
    source: dict[str, Any] = {
        "bag_path": relative_source_path(bag_path, source_root),
        "bag_sha256": bag_sha256,
        "bag_size_bytes": bag_path.stat().st_size,
        "capture_metadata": capture_metadata,
    }
    if declared_path is not None:
        source["declared_original_bag_path"] = declared_path
    return source, sidecars


def build_fixture(
    bag_path: Path,
    source_root: Path,
    output_dir: Path,
    blob_root: Path,
    inputs: RosbagPlaneSegInputs,
    pairing: PairingResult,
    selected_indices: Sequence[int],
    frames_per_bag: int | str,
    bag_sha256: str,
    parent_frame: str,
    child_frame: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    source, sidecars = build_source_metadata(
        bag_path, source_root, bag_sha256
    )
    frames = []
    blobs = []
    for selected_index, complete_pair_index in enumerate(selected_indices):
        frame, frame_blobs = export_frame(
            output_dir,
            blob_root,
            selected_index,
            pairing.complete_pairs[complete_pair_index],
        )
        frames.append(frame)
        blobs.extend(frame_blobs)

    requested = parse_frames_per_bag(frames_per_bag)
    return (
        {
            "frames": frames,
            "hashing": {
                "algorithm": "sha256",
                "binary_layer_hashes": "stored in layer metadata and manifest.json",
                "expected_quadrangles_canonicalization": (
                    CANONICAL_JSON_DESCRIPTION
                ),
                "fixture_file_hash_location": "manifest.json",
            },
            "pairing": pairing_metadata(
                inputs, pairing, parent_frame, child_frame
            ),
            "provenance": {
                "exporter": EXPORTER_PATH,
                "parser": PARSER_PATH,
                "selection": {
                    "algorithm": (
                        "evenly_spaced_complete_grid_map_quadrangles_pairs_"
                        "inclusive_v1"
                    ),
                    "requested_frame_count": "all" if requested == 0 else requested,
                    "selected_complete_pair_indices": list(selected_indices),
                    "selected_pair_indices": [
                        pairing.complete_pairs[index].pair_index
                        for index in selected_indices
                    ],
                    "source_complete_pair_count": len(pairing.complete_pairs),
                    "source_exact_pair_count": len(pairing.exact_pairs),
                },
                "sidecars": sidecars,
                "topics": {
                    "grid_map": DEFAULT_GRID_MAP_TOPIC,
                    "height_map_mode": DEFAULT_HEIGHT_MAP_MODE_TOPIC,
                    "height_map_mode_state": DEFAULT_HEIGHT_MAP_MODE_STATE_TOPIC,
                    "look_pose": DEFAULT_LOOK_POSE_TOPIC,
                    "quadrangles": DEFAULT_QUADRANGLES_TOPIC,
                    "tf": DEFAULT_TF_TOPIC,
                    "tf_static": DEFAULT_TF_STATIC_TOPIC,
                },
            },
            "schema": SCHEMA,
            "source": source,
            "tf_static": [
                transform_metadata(recorded)
                for recorded in pairing.static_transforms
            ],
        },
        blobs,
    )


def export_fixtures(
    bag_paths: Iterable[Path],
    output_dir: Path,
    source_root: Path | None = None,
    frames_per_bag: int | str = 3,
    parent_frame: str = DEFAULT_PARENT_FRAME,
    child_frame: str = DEFAULT_CHILD_FRAME,
) -> dict[str, Any]:
    bags = [Path(path) for path in bag_paths]
    if not bags:
        raise ValueError("at least one ROS bag is required")
    parse_frames_per_bag(frames_per_bag)
    for bag_path in bags:
        if not bag_path.is_file():
            raise FileNotFoundError(f"ROS bag does not exist: {bag_path}")

    source_root = source_root or default_source_root(bags)
    ordered_bags = sorted(
        bags, key=lambda path: relative_source_path(path, source_root)
    )
    fixture_names = [
        f"{path.stem}.plane_seg_paired.json" for path in ordered_bags
    ]
    blob_roots = [
        Path(f"{path.stem}.plane_seg_paired") for path in ordered_bags
    ]
    if len(set(fixture_names)) != len(fixture_names):
        raise ValueError("source bags produce duplicate fixture file names")

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_entries = []
    for bag_path, fixture_name, blob_root in zip(
        ordered_bags, fixture_names, blob_roots
    ):
        bag_sha256 = sha256_file(bag_path)
        inputs = read_rosbag_plane_seg_inputs(bag_path)
        pairing = pair_plane_seg_inputs(
            inputs, parent_frame=parent_frame, child_frame=child_frame
        )
        selected_indices = select_frame_indices(
            len(pairing.complete_pairs), frames_per_bag
        )
        fixture, blobs = build_fixture(
            bag_path=bag_path,
            source_root=source_root,
            output_dir=output_dir,
            blob_root=blob_root,
            inputs=inputs,
            pairing=pairing,
            selected_indices=selected_indices,
            frames_per_bag=frames_per_bag,
            bag_sha256=bag_sha256,
            parent_frame=parent_frame,
            child_frame=child_frame,
        )
        fixture_bytes = formatted_json_bytes(fixture)
        (output_dir / fixture_name).write_bytes(fixture_bytes)
        manifest_entries.append(
            {
                "blobs": blobs,
                "fixture_path": fixture_name,
                "fixture_sha256": sha256_bytes(fixture_bytes),
                "fixture_size_bytes": len(fixture_bytes),
                "selected_frame_count": len(fixture["frames"]),
                "source_bag_path": fixture["source"]["bag_path"],
                "source_bag_sha256": bag_sha256,
            }
        )

    manifest = {
        "fixtures": manifest_entries,
        "hashing": {
            "algorithm": "sha256",
            "fixture_file_bytes": "exact UTF-8 bytes including trailing newline",
            "layer_blob_bytes": "exact bytes copied from ROS Float32MultiArray.data",
        },
        "schema": MANIFEST_SCHEMA,
    }
    manifest_bytes = formatted_json_bytes(manifest)
    (output_dir / "manifest.json").write_bytes(manifest_bytes)
    (output_dir / "manifest.json.sha256").write_text(
        f"{sha256_bytes(manifest_bytes)}  manifest.json\n",
        encoding="ascii",
        newline="\n",
    )
    return manifest


def frames_per_bag_argument(value: str) -> int:
    try:
        return parse_frames_per_bag(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path, help="source ROS1 bag paths")
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="directory for paired JSON metadata and binary layer blobs",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        help="root used to store portable relative source paths",
    )
    parser.add_argument(
        "--frames-per-bag",
        type=frames_per_bag_argument,
        default=3,
        help=(
            "evenly spaced complete GridMap/quadrangles pairs to retain; "
            "use 0 or all for every pair"
        ),
    )
    parser.add_argument("--parent-frame", default=DEFAULT_PARENT_FRAME)
    parser.add_argument("--child-frame", default=DEFAULT_CHILD_FRAME)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    manifest = export_fixtures(
        bag_paths=args.bags,
        output_dir=args.output_dir,
        source_root=args.source_root,
        frames_per_bag=args.frames_per_bag,
        parent_frame=args.parent_frame,
        child_frame=args.child_frame,
    )
    total_frames = sum(
        entry["selected_frame_count"] for entry in manifest["fixtures"]
    )
    total_blob_bytes = sum(
        blob["size_bytes"]
        for entry in manifest["fixtures"]
        for blob in entry["blobs"]
    )
    print(
        f"exported {total_frames} GridMap/quadrangles paired frames and "
        f"{total_blob_bytes} layer bytes to {args.output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
