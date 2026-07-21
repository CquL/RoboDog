#!/usr/bin/env python3
"""Compare two X30 quadrangle analysis directories."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def numeric_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "minimum": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "maximum": max(values),
    }


def load_groups(analysis_dir: Path) -> list[dict[str, float | int]]:
    path = analysis_dir / "quadrangle_groups.csv"
    groups = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if int(row["group_index"]) == 0 or row["degenerate"] == "True":
                continue
            points = [
                (
                    float(row[f"p{index}_x"]),
                    float(row[f"p{index}_y"]),
                    float(row[f"p{index}_z"]),
                )
                for index in range(4)
            ]
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]
            z_values = [point[2] for point in points]
            groups.append(
                {
                    "frame_index": int(row["frame_index"]),
                    "center_x": statistics.fmean(x_values),
                    "center_y": statistics.fmean(y_values),
                    "mean_z": statistics.fmean(z_values),
                    "minimum_x": min(x_values),
                    "maximum_x": max(x_values),
                    "minimum_y": min(y_values),
                    "maximum_y": max(y_values),
                    "extent_x": max(x_values) - min(x_values),
                    "extent_y": max(y_values) - min(y_values),
                    "z_range": float(row["z_range_m"]),
                    "area": float(row["area_m2"]),
                }
            )
    return groups


def bin_center(value: float, width: float) -> float:
    return round(value / width) * width


def summarize_groups(
    groups: list[dict[str, float | int]],
    bin_width: float,
    horizontal_z_range: float,
) -> dict:
    by_frame: dict[int, list[dict[str, float | int]]] = defaultdict(list)
    for group in groups:
        by_frame[int(group["frame_index"])].append(group)

    horizontal = [
        group for group in groups if float(group["z_range"]) <= horizontal_z_range
    ]
    z_values = [float(group["mean_z"]) for group in groups]
    horizontal_z = [float(group["mean_z"]) for group in horizontal]
    histogram = Counter(bin_center(value, bin_width) for value in horizontal_z)
    dominant_z = histogram.most_common(1)[0][0] if histogram else None

    return {
        "frames": len(by_frame),
        "groups": len(groups),
        "groups_per_frame": numeric_summary(
            [float(len(frame_groups)) for frame_groups in by_frame.values()]
        ),
        "horizontal_groups": len(horizontal),
        "nonhorizontal_groups": len(groups) - len(horizontal),
        "mean_z_m": numeric_summary(z_values),
        "area_m2": numeric_summary([float(group["area"]) for group in groups]),
        "dominant_horizontal_z_bin_m": dominant_z,
        "top_horizontal_z_bins": [
            {"z_m": center, "groups": count}
            for center, count in histogram.most_common(12)
        ],
    }


def count_level_pairs(
    groups: list[dict[str, float | int]],
    target_height: float,
    tolerance: float,
) -> tuple[int, int]:
    by_frame: dict[int, list[float]] = defaultdict(list)
    for group in groups:
        by_frame[int(group["frame_index"])].append(float(group["mean_z"]))

    frame_count = 0
    pair_count = 0
    for z_values in by_frame.values():
        matches = 0
        for left_index, left in enumerate(z_values):
            for right in z_values[left_index + 1 :]:
                if abs(abs(left - right) - target_height) <= tolerance:
                    matches += 1
        frame_count += int(matches > 0)
        pair_count += matches
    return frame_count, pair_count


def group_overlaps_bounds(group: dict, bounds: dict) -> bool:
    return not (
        float(group["maximum_x"]) < float(bounds["x"][0])
        or float(group["minimum_x"]) > float(bounds["x"][1])
        or float(group["maximum_y"]) < float(bounds["y"][0])
        or float(group["minimum_y"]) > float(bounds["y"][1])
    )


def summarize_local_surface(
    groups: list[dict[str, float | int]],
    bounds: dict,
    expected_top_z: float,
    height_tolerance: float,
    horizontal_z_range: float,
) -> dict:
    selected = [
        group
        for group in groups
        if group_overlaps_bounds(group, bounds)
        and float(group["z_range"]) <= horizontal_z_range
        and abs(float(group["mean_z"]) - expected_top_z) <= height_tolerance
    ]
    frames = {int(group["frame_index"]) for group in selected}
    return {
        "groups": len(selected),
        "frames": len(frames),
        "center_x_m": numeric_summary(
            [float(group["center_x"]) for group in selected]
        ),
        "center_y_m": numeric_summary(
            [float(group["center_y"]) for group in selected]
        ),
        "top_z_m": numeric_summary(
            [float(group["mean_z"]) for group in selected]
        ),
        "top_z_error_m": numeric_summary(
            [float(group["mean_z"]) - expected_top_z for group in selected]
        ),
        "visible_extent_x_m": numeric_summary(
            [float(group["extent_x"]) for group in selected]
        ),
        "visible_extent_y_m": numeric_summary(
            [float(group["extent_y"]) for group in selected]
        ),
        "area_m2": numeric_summary(
            [float(group["area"]) for group in selected]
        ),
    }


def local_region_check(
    reference_groups: list[dict[str, float | int]],
    candidate_groups: list[dict[str, float | int]],
    gridmap_comparison_path: Path,
    expected_step_height: float,
    tolerance: float,
    horizontal_z_range: float,
    expected_step_depth: float | None,
    expected_step_width: float | None,
) -> dict:
    gridmap = json.loads(gridmap_comparison_path.read_text(encoding="utf-8"))
    component = gridmap.get("largest_component")
    if not component:
        raise ValueError("GridMap comparison contains no largest component")
    bounds = component["bounds_m"]
    reference_ground_z = component["layer_comparison"]["elevation"][
        "reference"
    ]["median"]
    expected_top_z = reference_ground_z + expected_step_height
    reference = summarize_local_surface(
        reference_groups,
        bounds,
        expected_top_z,
        tolerance,
        horizontal_z_range,
    )
    candidate = summarize_local_surface(
        candidate_groups,
        bounds,
        expected_top_z,
        tolerance,
        horizontal_z_range,
    )
    distinct = candidate["groups"] >= max(reference["groups"] + 3, 3)

    dimensions = {
        "expected_depth_x_m": expected_step_depth,
        "expected_width_y_m": expected_step_width,
        "observed_depth_x_m": (
            candidate["visible_extent_x_m"]["median"]
            if candidate["visible_extent_x_m"]
            else None
        ),
        "observed_width_y_m": (
            candidate["visible_extent_y_m"]["median"]
            if candidate["visible_extent_y_m"]
            else None
        ),
        "note": (
            "quadrangle extents are visible support patches, not guaranteed "
            "full physical dimensions"
        ),
    }
    if expected_step_depth is not None and dimensions["observed_depth_x_m"] is not None:
        dimensions["depth_error_m"] = (
            dimensions["observed_depth_x_m"] - expected_step_depth
        )
    if expected_step_width is not None and dimensions["observed_width_y_m"] is not None:
        dimensions["width_error_m"] = (
            dimensions["observed_width_y_m"] - expected_step_width
        )

    return {
        "gridmap_comparison": str(gridmap_comparison_path.resolve()),
        "bounds_m": bounds,
        "reference_ground_z_m": reference_ground_z,
        "expected_top_z_m": expected_top_z,
        "reference": reference,
        "candidate": candidate,
        "dimensions": dimensions,
        "distinct_local_surface_detected": distinct,
    }


def compare(
    reference_dir: Path,
    candidate_dir: Path,
    expected_step_height: float,
    tolerance: float,
    gridmap_comparison_path: Path | None = None,
    expected_step_depth: float | None = None,
    expected_step_width: float | None = None,
) -> dict:
    reference_groups = load_groups(reference_dir)
    candidate_groups = load_groups(candidate_dir)
    bin_width = 0.02
    horizontal_z_range = 0.03
    reference = summarize_groups(reference_groups, bin_width, horizontal_z_range)
    candidate = summarize_groups(candidate_groups, bin_width, horizontal_z_range)

    reference_ground = reference["dominant_horizontal_z_bin_m"]
    expected_center = (
        reference_ground + expected_step_height
        if reference_ground is not None
        else None
    )

    def elevated_count(groups):
        if expected_center is None:
            return 0
        return sum(
            abs(float(group["mean_z"]) - expected_center) <= tolerance
            and float(group["z_range"]) <= horizontal_z_range
            for group in groups
        )

    reference_pair_frames, reference_pairs = count_level_pairs(
        reference_groups, expected_step_height, tolerance
    )
    candidate_pair_frames, candidate_pairs = count_level_pairs(
        candidate_groups, expected_step_height, tolerance
    )
    reference_elevated = elevated_count(reference_groups)
    candidate_elevated = elevated_count(candidate_groups)
    global_distinct_signature = candidate_elevated >= max(reference_elevated + 3, 3)

    local_check = None
    if gridmap_comparison_path is not None:
        local_check = local_region_check(
            reference_groups,
            candidate_groups,
            gridmap_comparison_path,
            expected_step_height,
            tolerance,
            horizontal_z_range,
            expected_step_depth,
            expected_step_width,
        )
        local_check["reference"]["frame_detection_ratio"] = (
            local_check["reference"]["frames"] / reference["frames"]
            if reference["frames"]
            else 0.0
        )
        local_check["candidate"]["frame_detection_ratio"] = (
            local_check["candidate"]["frames"] / candidate["frames"]
            if candidate["frames"]
            else 0.0
        )
        distinct_signature = local_check["distinct_local_surface_detected"]
        decision_basis = "GridMap-localized support surface"
    else:
        distinct_signature = global_distinct_signature
        decision_basis = "global dominant horizontal level"

    return {
        "inputs": {
            "reference": str(reference_dir.resolve()),
            "candidate": str(candidate_dir.resolve()),
            "expected_step_height_m": expected_step_height,
            "height_tolerance_m": tolerance,
            "gridmap_comparison": (
                str(gridmap_comparison_path.resolve())
                if gridmap_comparison_path is not None
                else None
            ),
            "expected_step_depth_m": expected_step_depth,
            "expected_step_width_m": expected_step_width,
        },
        "reference": reference,
        "candidate": candidate,
        "expected_elevated_surface": {
            "reference_ground_z_m": reference_ground,
            "expected_center_z_m": expected_center,
            "reference_groups_in_band": reference_elevated,
            "candidate_groups_in_band": candidate_elevated,
            "global_distinct_signature": global_distinct_signature,
        },
        "local_region_check": local_check,
        "level_pair_check": {
            "reference_frames_with_matching_pair": reference_pair_frames,
            "reference_matching_pairs": reference_pairs,
            "candidate_frames_with_matching_pair": candidate_pair_frames,
            "candidate_matching_pairs": candidate_pairs,
        },
        "comparison": {
            "additional_groups": len(candidate_groups) - len(reference_groups),
            "decision_basis": decision_basis,
            "distinct_expected_elevated_surface_detected": distinct_signature,
            "interpretation": (
                "candidate contains an additional expected-height surface signature"
                if distinct_signature
                else "no distinct expected-height surface signature versus reference"
            ),
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--expected-step-height", type=float, required=True)
    parser.add_argument("--expected-step-depth", type=float)
    parser.add_argument("--expected-step-width", type=float)
    parser.add_argument("--gridmap-comparison", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = compare(
        args.reference,
        args.candidate,
        args.expected_step_height,
        args.tolerance,
        args.gridmap_comparison,
        args.expected_step_depth,
        args.expected_step_width,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
