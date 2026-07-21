#!/usr/bin/env python3
"""Analyze and compare factory X30 GridMap layers from ROS1 bag files."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import struct
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import analyze_x30_gridmap_baseline as wire


GRID_MAP_TOPIC = "/deeprobotics_local_height_map_mid360/height_map"
NUMERIC_LAYERS = ("elevation", "slope", "accessibility")


@dataclass(frozen=True)
class GridGeometry:
    size_x: int
    size_y: int
    resolution: float
    length_x: float
    length_y: float
    center_x: float
    center_y: float
    center_z: float
    yaw: float


@dataclass(frozen=True)
class CellStats:
    medians: tuple[float, ...]
    observation_counts: tuple[int, ...]


@dataclass
class CaptureAnalysis:
    bag_path: Path
    frame_count: int
    geometry: GridGeometry
    summary: dict
    layer_stats: dict[str, CellStats]
    elevation_frames: tuple[tuple[float, ...], ...]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def numeric_summary(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "minimum": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "maximum": max(values),
    }


def counter_to_dict(counter: Counter) -> dict[str, int]:
    return {str(key): value for key, value in sorted(counter.items())}


def quaternion_yaw(orientation: Sequence[float]) -> float:
    x, y, z, w = orientation
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def layer_shape(layer: wire.GridMapLayer) -> tuple[int, int]:
    if len(layer.dimensions) != 2:
        raise ValueError(
            f"GridMap layer {layer.name!r} has {len(layer.dimensions)} dimensions"
        )
    outer, inner = layer.dimensions
    if outer.label != "column_index" or inner.label != "row_index":
        raise ValueError(
            f"unsupported GridMap layout labels: {outer.label!r}, {inner.label!r}"
        )
    if outer.size <= 0 or inner.size <= 0:
        raise ValueError("GridMap dimensions must be positive")
    required = layer.data_offset + outer.size * inner.size
    if required > len(layer.values):
        raise ValueError(
            f"GridMap layer {layer.name!r} needs {required} values, "
            f"contains {len(layer.values)}"
        )
    return outer.size, inner.size


def geometry_from_frame(frame: wire.GridMapFrame) -> GridGeometry:
    if not frame.layers:
        raise ValueError("GridMap contains no layers")
    size_x, size_y = layer_shape(frame.layers[0])
    for layer in frame.layers[1:]:
        if layer_shape(layer) != (size_x, size_y):
            raise ValueError("GridMap layers do not share one matrix shape")
    expected_x = size_x * frame.resolution
    expected_y = size_y * frame.resolution
    if not math.isclose(expected_x, frame.length_x, abs_tol=1e-9):
        raise ValueError(
            f"GridMap length_x {frame.length_x} does not match "
            f"size_x * resolution {expected_x}"
        )
    if not math.isclose(expected_y, frame.length_y, abs_tol=1e-9):
        raise ValueError(
            f"GridMap length_y {frame.length_y} does not match "
            f"size_y * resolution {expected_y}"
        )
    return GridGeometry(
        size_x=size_x,
        size_y=size_y,
        resolution=frame.resolution,
        length_x=frame.length_x,
        length_y=frame.length_y,
        center_x=frame.position[0],
        center_y=frame.position[1],
        center_z=frame.position[2],
        yaw=quaternion_yaw(frame.orientation),
    )


def validate_frame_geometry(
    frame: wire.GridMapFrame,
    expected: GridGeometry,
) -> None:
    actual = geometry_from_frame(frame)
    if actual != expected:
        raise ValueError(f"GridMap geometry changed: {expected!r} -> {actual!r}")


def canonical_layer_values(
    frame: wire.GridMapFrame,
    layer_name: str,
) -> tuple[float, ...]:
    """Return logical GridMap cells in x-index + y-index * size_x order."""
    layer = frame.layer(layer_name)
    size_x, size_y = layer_shape(layer)
    if frame.outer_start_index >= size_x or frame.inner_start_index >= size_y:
        raise ValueError(
            "GridMap circular-buffer start index lies outside the matrix"
        )

    result = [math.nan] * (size_x * size_y)
    for y_index in range(size_y):
        buffer_y = (y_index + frame.inner_start_index) % size_y
        for x_index in range(size_x):
            buffer_x = (x_index + frame.outer_start_index) % size_x
            raw_index = (
                layer.data_offset
                + buffer_x
                + buffer_y * size_x
            )
            result[x_index + y_index * size_x] = layer.values[raw_index]
    return tuple(result)


def cell_position(
    geometry: GridGeometry,
    x_index: int,
    y_index: int,
) -> tuple[float, float]:
    local_x = geometry.length_x / 2.0 - (x_index + 0.5) * geometry.resolution
    local_y = geometry.length_y / 2.0 - (y_index + 0.5) * geometry.resolution
    cos_yaw = math.cos(geometry.yaw)
    sin_yaw = math.sin(geometry.yaw)
    return (
        geometry.center_x + cos_yaw * local_x - sin_yaw * local_y,
        geometry.center_y + sin_yaw * local_x + cos_yaw * local_y,
    )


def summarize_numeric_layer(
    frames: Sequence[tuple[float, ...]],
) -> tuple[CellStats, dict]:
    if not frames:
        raise ValueError("cannot summarize an empty layer capture")
    cell_count = len(frames[0])
    if any(len(frame) != cell_count for frame in frames):
        raise ValueError("GridMap layer frame sizes differ")

    medians = []
    observation_counts = []
    for cell_index in range(cell_count):
        values = [
            frame[cell_index]
            for frame in frames
            if math.isfinite(frame[cell_index])
        ]
        observation_counts.append(len(values))
        medians.append(statistics.median(values) if values else math.nan)

    frame_finite_counts = []
    finite_minimum = math.inf
    finite_maximum = -math.inf
    finite_sum = 0.0
    finite_count = 0
    rounded_distribution: Counter[float] = Counter()
    for frame in frames:
        count = 0
        for value in frame:
            if not math.isfinite(value):
                continue
            count += 1
            finite_count += 1
            finite_sum += value
            finite_minimum = min(finite_minimum, value)
            finite_maximum = max(finite_maximum, value)
            rounded_distribution[round(value, 6)] += 1
        frame_finite_counts.append(float(count))

    summary = {
        "finite_values": finite_count,
        "finite_fraction": finite_count / (len(frames) * cell_count),
        "finite_cells_per_frame": numeric_summary(frame_finite_counts),
        "minimum": finite_minimum if finite_count else None,
        "mean": finite_sum / finite_count if finite_count else None,
        "maximum": finite_maximum if finite_count else None,
        "top_rounded_values": [
            {"value": value, "count": count}
            for value, count in rounded_distribution.most_common(12)
        ],
    }
    return (
        CellStats(tuple(medians), tuple(observation_counts)),
        summary,
    )


def summarize_color_layer(frames: Sequence[wire.GridMapFrame]) -> dict:
    bit_patterns: Counter[int] = Counter()
    for frame in frames:
        for value in frame.layer("color").values:
            bit_patterns[struct.unpack("<I", struct.pack("<f", value))[0]] += 1
    return {
        "encoding": "packed uint32 color bits stored in float32",
        "values": sum(bit_patterns.values()),
        "unique_bit_patterns": len(bit_patterns),
        "top_bit_patterns": [
            {"uint32": value, "hex": f"0x{value:08x}", "count": count}
            for value, count in bit_patterns.most_common(12)
        ],
    }


def write_csv(path: Path, rows: Iterable[dict]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_capture_cells(
    path: Path,
    geometry: GridGeometry,
    layer_stats: dict[str, CellStats],
) -> None:
    rows = []
    for y_index in range(geometry.size_y):
        for x_index in range(geometry.size_x):
            cell_index = x_index + y_index * geometry.size_x
            x, y = cell_position(geometry, x_index, y_index)
            row = {
                "cell_index": cell_index,
                "x_index": x_index,
                "y_index": y_index,
                "x_m": x,
                "y_m": y,
            }
            for layer_name, stats in layer_stats.items():
                row[f"{layer_name}_observations"] = stats.observation_counts[cell_index]
                row[f"{layer_name}_median"] = stats.medians[cell_index]
            rows.append(row)
    write_csv(path, rows)


def analyze_capture(
    bag_path: Path,
    output_dir: Path,
    label: str,
) -> CaptureAnalysis:
    frames = wire.read_rosbag_grid_maps(bag_path, GRID_MAP_TOPIC)
    if not frames:
        raise ValueError(f"no {GRID_MAP_TOPIC} messages in {bag_path}")

    geometry = geometry_from_frame(frames[0])
    expected_layers = tuple(layer.name for layer in frames[0].layers)
    expected_basic_layers = frames[0].basic_layers
    for frame in frames:
        validate_frame_geometry(frame, geometry)
        if tuple(layer.name for layer in frame.layers) != expected_layers:
            raise ValueError("GridMap layer order changed during capture")
        if frame.basic_layers != expected_basic_layers:
            raise ValueError("GridMap basic_layers changed during capture")

    canonical: dict[str, tuple[tuple[float, ...], ...]] = {}
    layer_stats: dict[str, CellStats] = {}
    layer_summaries = {}
    for layer_name in NUMERIC_LAYERS:
        layer_frames = tuple(
            canonical_layer_values(frame, layer_name) for frame in frames
        )
        canonical[layer_name] = layer_frames
        stats, summary = summarize_numeric_layer(layer_frames)
        layer_stats[layer_name] = stats
        layer_summaries[layer_name] = summary
    layer_summaries["color"] = summarize_color_layer(frames)

    intervals_ms = [
        (right.stamp_ns - left.stamp_ns) / 1_000_000.0
        for left, right in zip(frames, frames[1:])
    ]
    summary = {
        "input": {
            "bag": str(bag_path.resolve()),
            "sha256": sha256_file(bag_path),
            "topic": GRID_MAP_TOPIC,
        },
        "frames": len(frames),
        "sequence": {
            "first": frames[0].sequence,
            "last": frames[-1].sequence,
            "unique": len({frame.sequence for frame in frames}),
        },
        "source_interval_ms": numeric_summary(intervals_ms),
        "geometry": {
            "frame_ids": dict(Counter(frame.frame_id for frame in frames)),
            "resolution_m": geometry.resolution,
            "size": [geometry.size_x, geometry.size_y],
            "length_m": [geometry.length_x, geometry.length_y],
            "center_m": [geometry.center_x, geometry.center_y, geometry.center_z],
            "yaw_rad": geometry.yaw,
            "outer_inner_start_indices": counter_to_dict(
                Counter(
                    (frame.outer_start_index, frame.inner_start_index)
                    for frame in frames
                )
            ),
        },
        "layers": list(expected_layers),
        "basic_layers": list(expected_basic_layers),
        "layer_summaries": layer_summaries,
        "storage_mapping": {
            "logical_linear_index": "x_index + y_index * size_x",
            "buffer_x": "(x_index + outer_start_index) % size_x",
            "buffer_y": "(y_index + inner_start_index) % size_y",
            "raw_linear_index": "data_offset + buffer_x + buffer_y * size_x",
            "position_x": "center_x + length_x/2 - (x_index+0.5)*resolution",
            "position_y": "center_y + length_y/2 - (y_index+0.5)*resolution",
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / f"{label}_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_capture_cells(
        output_dir / f"{label}_cells.csv",
        geometry,
        layer_stats,
    )
    return CaptureAnalysis(
        bag_path=bag_path,
        frame_count=len(frames),
        geometry=geometry,
        summary=summary,
        layer_stats=layer_stats,
        elevation_frames=canonical["elevation"],
    )


def connected_components(cells: set[tuple[int, int]]) -> list[list[tuple[int, int]]]:
    remaining = set(cells)
    components = []
    while remaining:
        stack = [remaining.pop()]
        component = []
        while stack:
            cell = stack.pop()
            component.append(cell)
            neighbors = (
                (cell[0] - 1, cell[1]),
                (cell[0] + 1, cell[1]),
                (cell[0], cell[1] - 1),
                (cell[0], cell[1] + 1),
            )
            for neighbor in neighbors:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return sorted(components, key=len, reverse=True)


def finite_values(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def component_summary(
    component_id: int,
    cells: Sequence[tuple[int, int]],
    reference: CaptureAnalysis,
    candidate: CaptureAnalysis,
    elevation_threshold: float,
) -> dict:
    geometry = reference.geometry
    cell_indices = [x + y * geometry.size_x for x, y in cells]
    positions = [cell_position(geometry, x, y) for x, y in cells]
    reference_elevation = reference.layer_stats["elevation"].medians
    candidate_elevation = candidate.layer_stats["elevation"].medians
    deltas = [
        candidate_elevation[index] - reference_elevation[index]
        for index in cell_indices
    ]

    frame_hit_counts = []
    for frame in candidate.elevation_frames:
        frame_hit_counts.append(
            sum(
                math.isfinite(frame[index])
                and math.isfinite(reference_elevation[index])
                and frame[index] - reference_elevation[index] >= elevation_threshold
                for index in cell_indices
            )
        )
    required_frame_cells = max(1, math.ceil(len(cells) * 0.2))

    layer_comparison = {}
    for layer_name in NUMERIC_LAYERS:
        reference_values = finite_values(
            reference.layer_stats[layer_name].medians[index]
            for index in cell_indices
        )
        candidate_values = finite_values(
            candidate.layer_stats[layer_name].medians[index]
            for index in cell_indices
        )
        paired_deltas = finite_values(
            candidate.layer_stats[layer_name].medians[index]
            - reference.layer_stats[layer_name].medians[index]
            for index in cell_indices
            if math.isfinite(reference.layer_stats[layer_name].medians[index])
            and math.isfinite(candidate.layer_stats[layer_name].medians[index])
        )
        layer_comparison[layer_name] = {
            "reference": numeric_summary(reference_values),
            "candidate": numeric_summary(candidate_values),
            "candidate_minus_reference": numeric_summary(paired_deltas),
        }

    x_values = [position[0] for position in positions]
    y_values = [position[1] for position in positions]
    half_cell = geometry.resolution / 2.0
    return {
        "component_id": component_id,
        "cells": len(cells),
        "area_m2": len(cells) * geometry.resolution * geometry.resolution,
        "centroid_m": [statistics.fmean(x_values), statistics.fmean(y_values)],
        "bounds_m": {
            "x": [min(x_values) - half_cell, max(x_values) + half_cell],
            "y": [min(y_values) - half_cell, max(y_values) + half_cell],
        },
        "elevation_delta_m": numeric_summary(deltas),
        "layer_comparison": layer_comparison,
        "temporal_detection": {
            "minimum_changed_cells_per_frame": required_frame_cells,
            "frames_detected": sum(
                hits >= required_frame_cells for hits in frame_hit_counts
            ),
            "frames_total": candidate.frame_count,
            "detection_ratio": sum(
                hits >= required_frame_cells for hits in frame_hit_counts
            ) / candidate.frame_count,
            "changed_cells_per_frame": numeric_summary(
                [float(value) for value in frame_hit_counts]
            ),
        },
    }


def compare_captures(
    reference: CaptureAnalysis,
    candidate: CaptureAnalysis,
    output_dir: Path,
    elevation_threshold: float,
    min_observation_ratio: float,
    min_component_cells: int,
) -> dict:
    if reference.geometry != candidate.geometry:
        raise ValueError("reference and candidate GridMap geometries differ")
    geometry = reference.geometry
    minimum_reference_observations = math.ceil(
        reference.frame_count * min_observation_ratio
    )
    minimum_candidate_observations = math.ceil(
        candidate.frame_count * min_observation_ratio
    )
    reference_stats = reference.layer_stats["elevation"]
    candidate_stats = candidate.layer_stats["elevation"]

    comparable_indices = []
    positive_cells: set[tuple[int, int]] = set()
    absolute_changed = 0
    for cell_index, (reference_value, candidate_value) in enumerate(
        zip(reference_stats.medians, candidate_stats.medians)
    ):
        if (
            reference_stats.observation_counts[cell_index]
            < minimum_reference_observations
            or candidate_stats.observation_counts[cell_index]
            < minimum_candidate_observations
            or not math.isfinite(reference_value)
            or not math.isfinite(candidate_value)
        ):
            continue
        comparable_indices.append(cell_index)
        delta = candidate_value - reference_value
        absolute_changed += int(abs(delta) >= elevation_threshold)
        if delta >= elevation_threshold:
            positive_cells.add(
                (cell_index % geometry.size_x, cell_index // geometry.size_x)
            )

    components = connected_components(positive_cells)
    component_summaries = [
        component_summary(
            component_id,
            component,
            reference,
            candidate,
            elevation_threshold,
        )
        for component_id, component in enumerate(components, start=1)
    ]
    cell_to_component = {
        cell: component_id
        for component_id, component in enumerate(components, start=1)
        for cell in component
    }

    changed_rows = []
    for x_index, y_index in sorted(positive_cells, key=lambda cell: (cell[1], cell[0])):
        cell_index = x_index + y_index * geometry.size_x
        x, y = cell_position(geometry, x_index, y_index)
        changed_rows.append(
            {
                "component_id": cell_to_component[(x_index, y_index)],
                "cell_index": cell_index,
                "x_index": x_index,
                "y_index": y_index,
                "x_m": x,
                "y_m": y,
                "reference_elevation_m": reference_stats.medians[cell_index],
                "candidate_elevation_m": candidate_stats.medians[cell_index],
                "elevation_delta_m": (
                    candidate_stats.medians[cell_index]
                    - reference_stats.medians[cell_index]
                ),
                "reference_observations": reference_stats.observation_counts[cell_index],
                "candidate_observations": candidate_stats.observation_counts[cell_index],
            }
        )
    write_csv(output_dir / "positive_elevation_changes.csv", changed_rows)
    write_csv(
        output_dir / "elevation_components.csv",
        (
            {
                "component_id": component["component_id"],
                "cells": component["cells"],
                "area_m2": component["area_m2"],
                "centroid_x_m": component["centroid_m"][0],
                "centroid_y_m": component["centroid_m"][1],
                "min_x_m": component["bounds_m"]["x"][0],
                "max_x_m": component["bounds_m"]["x"][1],
                "min_y_m": component["bounds_m"]["y"][0],
                "max_y_m": component["bounds_m"]["y"][1],
                "median_elevation_delta_m": component["elevation_delta_m"]["median"],
                "detection_ratio": component["temporal_detection"]["detection_ratio"],
            }
            for component in component_summaries
        ),
    )

    largest = component_summaries[0] if component_summaries else None
    distinct_surface = bool(
        largest
        and largest["cells"] >= min_component_cells
        and largest["elevation_delta_m"]["median"] >= elevation_threshold
    )
    result = {
        "inputs": {
            "reference_bag": str(reference.bag_path.resolve()),
            "candidate_bag": str(candidate.bag_path.resolve()),
            "elevation_threshold_m": elevation_threshold,
            "minimum_observation_ratio": min_observation_ratio,
            "minimum_component_cells": min_component_cells,
        },
        "comparison": {
            "comparable_cells": len(comparable_indices),
            "absolute_changed_cells": absolute_changed,
            "positive_changed_cells": len(positive_cells),
            "connected_components": len(components),
            "largest_component_cells": largest["cells"] if largest else 0,
            "distinct_elevated_surface_detected": distinct_surface,
            "interpretation": (
                "candidate contains a persistent elevated surface"
                if distinct_surface
                else "no persistent elevated surface above the configured threshold"
            ),
        },
        "largest_component": largest,
        "components": component_summaries,
        "cross_check": {
            "expected_object_region_from_quadrangles_m": {
                "center": [1.0725, -0.0527],
                "note": "derived independently from /plane_seg/quadrangels",
            }
        },
    }
    (output_dir / "gridmap_comparison.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-bag", type=Path, required=True)
    parser.add_argument("--candidate-bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--elevation-threshold", type=float, default=0.08)
    parser.add_argument("--min-observation-ratio", type=float, default=0.5)
    parser.add_argument("--min-component-cells", type=int, default=20)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 0.0 < args.min_observation_ratio <= 1.0:
        raise ValueError("--min-observation-ratio must be in (0, 1]")
    if args.elevation_threshold <= 0.0:
        raise ValueError("--elevation-threshold must be positive")
    if args.min_component_cells <= 0:
        raise ValueError("--min-component-cells must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reference = analyze_capture(
        args.reference_bag,
        args.output_dir,
        "reference",
    )
    candidate = analyze_capture(
        args.candidate_bag,
        args.output_dir,
        "candidate",
    )
    result = compare_captures(
        reference,
        candidate,
        args.output_dir,
        args.elevation_threshold,
        args.min_observation_ratio,
        args.min_component_cells,
    )
    print(json.dumps(result["comparison"], indent=2, ensure_ascii=False))
    if result["largest_component"]:
        print(
            json.dumps(
                {"largest_component": result["largest_component"]},
                indent=2,
                ensure_ascii=False,
            )
        )
    print(f"Results: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
