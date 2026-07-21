#!/usr/bin/env python3
"""Export compact deterministic plane_seg regression fixtures from ROS1 bags."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
from typing import Any, Iterable, Sequence

from analyze_x30_gridmap_baseline import PointCloudFrame, read_rosbag_quadrangles


SCHEMA = "x30-plane-seg-regression-fixture/v1"
MANIFEST_SCHEMA = "x30-plane-seg-regression-manifest/v1"
DEFAULT_TOPIC = "/plane_seg/quadrangels"
EXPORTER_PATH = "x30_livox_ros2_transfer/tools/export_x30_plane_seg_fixtures.py"
PARSER_PATH = "x30_livox_ros2_transfer/tools/analyze_x30_gridmap_baseline.py"
CANONICAL_JSON_DESCRIPTION = (
    "UTF-8 JSON with sorted keys and separators ',' and ':', without NaN"
)
SHA256_PATTERN = re.compile(r"^[0-9a-fA-F]{64}$")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def formatted_json_bytes(value: Any) -> bytes:
    return (
        json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def select_frame_indices(frame_count: int, requested_count: int) -> list[int]:
    if frame_count < 1:
        raise ValueError("cannot select fixtures from an empty topic")
    if requested_count < 1:
        raise ValueError("frames per bag must be at least one")
    if requested_count >= frame_count:
        return list(range(frame_count))
    if requested_count == 1:
        return [frame_count // 2]
    return [
        index * (frame_count - 1) // (requested_count - 1)
        for index in range(requested_count)
    ]


def relative_source_path(path: Path, source_root: Path) -> str:
    resolved_path = path.resolve()
    resolved_root = source_root.resolve()
    try:
        relative_path = resolved_path.relative_to(resolved_root)
    except ValueError as error:
        raise ValueError(
            f"source {resolved_path} is outside source root {resolved_root}"
        ) from error
    return relative_path.as_posix()


def parse_capture_metadata(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    metadata: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator or not key:
            raise ValueError(
                f"invalid metadata line {line_number} in {path}: {raw_line!r}"
            )
        if key in metadata:
            raise ValueError(f"duplicate metadata key {key!r} in {path}")
        metadata[key] = value
    return metadata


def parse_declared_bag_sha256(path: Path) -> tuple[str, str | None]:
    tokens = path.read_text(encoding="utf-8").strip().split(maxsplit=1)
    if not tokens or not SHA256_PATTERN.fullmatch(tokens[0]):
        raise ValueError(f"invalid SHA-256 sidecar: {path}")
    declared_path = tokens[1].lstrip("* ") if len(tokens) == 2 else None
    return tokens[0].lower(), declared_path or None


def sidecar_metadata(
    bag_path: Path,
    source_root: Path,
) -> tuple[list[dict[str, Any]], dict[str, str], str | None, str | None]:
    sidecars = []
    capture_metadata_path = bag_path.with_suffix(".metadata.txt")
    info_path = bag_path.with_suffix(".info.yaml")
    bag_sha_path = bag_path.with_suffix(".bag.sha256")

    capture_metadata = parse_capture_metadata(capture_metadata_path)
    declared_bag_sha256 = None
    declared_bag_path = None
    for role, path in (
        ("bag_sha256", bag_sha_path),
        ("capture_metadata", capture_metadata_path),
        ("rosbag_info", info_path),
    ):
        if not path.exists():
            continue
        sidecars.append(
            {
                "path": relative_source_path(path, source_root),
                "role": role,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
        if role == "bag_sha256":
            declared_bag_sha256, declared_bag_path = parse_declared_bag_sha256(path)

    return sidecars, capture_metadata, declared_bag_sha256, declared_bag_path


def frame_payload(frame: PointCloudFrame, source_frame_index: int) -> dict[str, Any]:
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
        "source_frame_index": source_frame_index,
    }


def fixture_frame(frame: PointCloudFrame, source_frame_index: int) -> dict[str, Any]:
    payload = frame_payload(frame, source_frame_index)
    return {
        "payload": payload,
        "payload_sha256": sha256_bytes(canonical_json_bytes(payload)),
    }


def build_fixture(
    bag_path: Path,
    source_root: Path,
    source_bag_sha256: str,
    frames: Sequence[PointCloudFrame],
    frame_indices: Sequence[int],
    frames_per_bag: int,
    topic: str,
) -> dict[str, Any]:
    sidecars, capture_metadata, declared_sha256, declared_path = sidecar_metadata(
        bag_path, source_root
    )
    if declared_sha256 is not None and declared_sha256 != source_bag_sha256:
        raise ValueError(
            f"SHA-256 mismatch for {bag_path}: "
            f"sidecar={declared_sha256}, actual={source_bag_sha256}"
        )

    source: dict[str, Any] = {
        "bag_path": relative_source_path(bag_path, source_root),
        "bag_sha256": source_bag_sha256,
        "bag_size_bytes": bag_path.stat().st_size,
        "capture_metadata": capture_metadata,
    }
    if declared_path is not None:
        source["declared_original_bag_path"] = declared_path

    return {
        "frames": [fixture_frame(frames[index], index) for index in frame_indices],
        "hashing": {
            "algorithm": "sha256",
            "frame_payload_canonicalization": CANONICAL_JSON_DESCRIPTION,
            "fixture_file_hash_location": "manifest.json",
        },
        "provenance": {
            "exporter": EXPORTER_PATH,
            "parser": PARSER_PATH,
            "selection": {
                "algorithm": "evenly_spaced_inclusive_v1",
                "requested_frame_count": frames_per_bag,
                "selected_frame_indices": list(frame_indices),
                "source_frame_count": len(frames),
            },
            "sidecars": sidecars,
            "topic": topic,
        },
        "schema": SCHEMA,
        "source": source,
    }


def default_source_root(bags: Sequence[Path]) -> Path:
    parents = [str(path.resolve().parent) for path in bags]
    return Path(os.path.commonpath(parents))


def export_fixtures(
    bag_paths: Iterable[Path],
    output_dir: Path,
    source_root: Path | None = None,
    frames_per_bag: int = 3,
    topic: str = DEFAULT_TOPIC,
) -> dict[str, Any]:
    bags = [Path(path) for path in bag_paths]
    if not bags:
        raise ValueError("at least one ROS bag is required")
    if frames_per_bag < 1:
        raise ValueError("frames per bag must be at least one")
    for bag_path in bags:
        if not bag_path.is_file():
            raise FileNotFoundError(f"ROS bag does not exist: {bag_path}")

    source_root = source_root or default_source_root(bags)
    ordered_bags = sorted(
        bags, key=lambda path: relative_source_path(path, source_root)
    )
    output_names = [f"{path.stem}.plane_seg.json" for path in ordered_bags]
    if len(set(output_names)) != len(output_names):
        raise ValueError("source bags produce duplicate fixture file names")

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_entries = []
    for bag_path, output_name in zip(ordered_bags, output_names):
        source_bag_sha256 = sha256_file(bag_path)
        frames, _mode_values, _look_poses = read_rosbag_quadrangles(
            bag_path, topic=topic
        )
        frame_indices = select_frame_indices(len(frames), frames_per_bag)
        fixture = build_fixture(
            bag_path=bag_path,
            source_root=source_root,
            source_bag_sha256=source_bag_sha256,
            frames=frames,
            frame_indices=frame_indices,
            frames_per_bag=frames_per_bag,
            topic=topic,
        )
        fixture_bytes = formatted_json_bytes(fixture)
        (output_dir / output_name).write_bytes(fixture_bytes)
        manifest_entries.append(
            {
                "fixture_path": output_name,
                "fixture_sha256": sha256_bytes(fixture_bytes),
                "fixture_size_bytes": len(fixture_bytes),
                "frame_payload_sha256": [
                    frame["payload_sha256"] for frame in fixture["frames"]
                ],
                "source_bag_path": fixture["source"]["bag_path"],
                "source_bag_sha256": source_bag_sha256,
            }
        )

    manifest = {
        "fixtures": manifest_entries,
        "hashing": {
            "algorithm": "sha256",
            "fixture_file_bytes": "exact UTF-8 bytes including trailing newline",
        },
        "schema": MANIFEST_SCHEMA,
    }
    manifest_bytes = formatted_json_bytes(manifest)
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_bytes(manifest_bytes)
    (output_dir / "manifest.json.sha256").write_text(
        f"{sha256_bytes(manifest_bytes)}  manifest.json\n",
        encoding="ascii",
        newline="\n",
    )
    return manifest


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least one")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path, help="source ROS1 bag paths")
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="directory for JSON fixtures and manifest",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        help="root used to store portable relative source paths",
    )
    parser.add_argument(
        "--frames-per-bag",
        type=positive_int,
        default=3,
        help="number of evenly spaced frames to retain (default: 3)",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"PointCloud2 topic to export (default: {DEFAULT_TOPIC})",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    manifest = export_fixtures(
        bag_paths=args.bags,
        output_dir=args.output_dir,
        source_root=args.source_root,
        frames_per_bag=args.frames_per_bag,
        topic=args.topic,
    )
    total_bytes = sum(
        entry["fixture_size_bytes"] for entry in manifest["fixtures"]
    )
    print(
        f"exported {len(manifest['fixtures'])} fixtures "
        f"({total_bytes} bytes) to {args.output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
