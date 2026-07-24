#!/usr/bin/env python3
"""Compare X30 replay candidates with recorded factory terrain quadrangles.

The tool is deliberately offline. It consumes the existing schema-v1 replay
pack and replay_fixture_test JSONL, reproduces candidateTopRectangle geometry,
mirrors the factory post-processing stages recovered from static analysis, and
writes deterministic per-stage diagnostics. Unrecovered suffix stages remain
explicitly outside this tool's parity claim.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import struct
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import analyze_x30_quadrangle_parity as parity


SCHEMA_NAME = "x30_quadrangle_match_diagnostic_v1"
KNOWN_FRAME_132_STAMP_NS = 10_428_477_123_846
FACTORY_STAIR_POSE_MINIMUM_LONG_EDGE_M = 0.8
FACTORY_STAIR_POSE_MINIMUM_SHORT_EDGE_M = 0.1
FACTORY_POSE_CONSENSUS_COSINE = 0.939692620786
FACTORY_CORRECT_MINIMUM_SIZE = 4
FACTORY_CUT_MINIMUM_REMAINING_DEPTH_M = 0.05
FACTORY_CUT_STEP_M = 0.03
FACTORY_CUT_REMOVED_POINT_RATIO = 0.261
FACTORY_CUT_MAXIMUM_REMOVED_POINTS = 9
FACTORY_BOUNDARY_SAMPLE_STEP_M = 0.03
FACTORY_BOUNDARY_SAMPLE_COUNT_EPSILON = 0.0001
FACTORY_MERGE_MAXIMUM_HEIGHT_DIFFERENCE_M = 0.061
FACTORY_MERGE_DIRECTION_COSINE = 0.965925826289
FACTORY_MERGE_MAXIMUM_DISTANCE_M = 0.3
FACTORY_MERGE_MAXIMUM_ALONG_STAIR_DISTANCE_M = 0.1
FACTORY_SIDE_DISTANCE_THRESHOLD_M = 0.22
FACTORY_SORT_MAXIMUM_PRIMARY_SHORT_EDGE_M = 0.45
FACTORY_SORT_MINIMUM_PRIMARY_LONG_EDGE_M = 0.50
FACTORY_REFERENCE_DISTANCE_SAMPLE_STEP_M = 0.05
FACTORY_REFERENCE_DISTANCE_SAMPLE_COUNT_EPSILON = 0.0001
FACTORY_REFERENCE_MAXIMUM_CANDIDATE_DISTANCE_M = 0.30
FACTORY_REFERENCE_NEAR_DISTANCE_M = 0.18
FACTORY_REPAIR_MINIMUM_OUTPUT_EXTENT_M = 0.08
FACTORY_REPAIR_MINIMUM_FORWARD_EXTENT_M = 0.10
FACTORY_WIDE_INITIAL_HEIGHT_TOLERANCE_M = 0.25
FACTORY_WIDE_HEIGHT_TOLERANCE_STEP_M = 0.10
FACTORY_WIDE_MAXIMUM_ATTEMPTED_HEIGHT_TOLERANCE_M = 2.95
FACTORY_DEGENERATE_FIRST_EDGE_SQUARED_M2 = 1.0e-6
Point = tuple[float, float, float]


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


class DiagnosticError(ValueError):
    """Raised when diagnostic configuration or geometry is invalid."""


@dataclass(frozen=True)
class Thresholds:
    center_xy_m: float = 0.35
    height_m: float = 0.12
    area_relative: float = 0.80
    vertex_rms_m: float = 0.45
    minimum_edge_m: float = 1.0e-6
    merge_height_m: float = 0.08
    merge_center_xy_m: float = 0.45
    merge_area_relative: float = 0.40
    merge_gap_xy_m: float = 0.12
    max_merge_clues_per_frame: int = 8

    def validate(self) -> None:
        positive = {
            "center_xy_m": self.center_xy_m,
            "height_m": self.height_m,
            "area_relative": self.area_relative,
            "vertex_rms_m": self.vertex_rms_m,
            "minimum_edge_m": self.minimum_edge_m,
            "merge_height_m": self.merge_height_m,
            "merge_center_xy_m": self.merge_center_xy_m,
            "merge_area_relative": self.merge_area_relative,
            "merge_gap_xy_m": self.merge_gap_xy_m,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise DiagnosticError(f"{name} must be finite and positive")
        if self.max_merge_clues_per_frame < 0:
            raise DiagnosticError("max_merge_clues_per_frame must be non-negative")

    def as_json(self) -> dict[str, Any]:
        return {
            "center_xy_m": self.center_xy_m,
            "height_m": self.height_m,
            "area_relative": self.area_relative,
            "vertex_rms_m": self.vertex_rms_m,
            "minimum_edge_m": self.minimum_edge_m,
            "merge_height_m": self.merge_height_m,
            "merge_center_xy_m": self.merge_center_xy_m,
            "merge_area_relative": self.merge_area_relative,
            "merge_gap_xy_m": self.merge_gap_xy_m,
            "max_merge_clues_per_frame": self.max_merge_clues_per_frame,
        }


@dataclass(frozen=True)
class Rectangle:
    points: tuple[Point, Point, Point, Point]
    center: Point
    area_3d_m2: float
    signed_xy_area_m2: float


@dataclass(frozen=True)
class CandidateTop:
    candidate_index: int
    success: bool
    diagnostic: str
    rectangle: Optional[Rectangle]
    hull_point_count: int
    hull_area_3d_m2: float
    size: Point
    translation: Point
    hull: tuple[Point, ...]
    contained_points: tuple[Point, ...]
    contained_points_are_recorded: bool = False


def _add(left: Point, right: Point) -> Point:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _subtract(left: Point, right: Point) -> Point:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _scale(point: Point, factor: float) -> Point:
    return tuple(value * factor for value in point)  # type: ignore[return-value]


def _dot(left: Point, right: Point) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(point: Point) -> float:
    return math.sqrt(_dot(point, point))


def _distance(left: Point, right: Point) -> float:
    return _norm(_subtract(left, right))


def _distance_xy(left: Point, right: Point) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _center(points: Sequence[Point]) -> Point:
    inverse = 1.0 / len(points)
    return (
        math.fsum(point[0] for point in points) * inverse,
        math.fsum(point[1] for point in points) * inverse,
        math.fsum(point[2] for point in points) * inverse,
    )


def _cross(left: Point, right: Point) -> Point:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


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
        area_vector = _add(area_vector, _cross(first, second))
    return 0.5 * _norm(area_vector)


def _signed_xy_area(points: Sequence[Point]) -> float:
    return 0.5 * math.fsum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, (*points[1:], points[0]))
    )


def _rectangle(points: Sequence[Point]) -> Rectangle:
    if len(points) != 4:
        raise DiagnosticError("a quadrangle must contain exactly four points")
    fixed = tuple(points)
    return Rectangle(
        points=fixed,  # type: ignore[arg-type]
        center=_center(fixed),
        area_3d_m2=_quadrangle_area(fixed),
        signed_xy_area_m2=_signed_xy_area(fixed),
    )


def _rotate_by_quaternion(point: Point, quaternion_xyzw: Sequence[float]) -> Point:
    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise DiagnosticError("candidate quaternion is not normalizable")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    vector = (x, y, z)
    t = _scale(_cross(vector, point), 2.0)
    return _add(point, _add(_scale(t, w), _cross(vector, t)))


def candidate_top_rectangle(
    candidate: parity.Candidate,
    candidate_index: int,
    minimum_edge_m: float = 1.0e-6,
) -> CandidateTop:
    """Python equivalent of x30_plane_seg_core::candidateTopRectangle."""

    hull_area = _polygon_area(candidate.hull)
    base = {
        "candidate_index": candidate_index,
        "hull_point_count": len(candidate.hull),
        "hull_area_3d_m2": hull_area,
        "size": candidate.size,
        "translation": candidate.translation,
        "hull": candidate.hull,
        "contained_points": candidate.contained_points,
        "contained_points_are_recorded": bool(candidate.contained_points),
    }
    if not math.isfinite(minimum_edge_m) or minimum_edge_m <= 0.0:
        raise DiagnosticError("minimum_edge_m must be finite and positive")
    if not all(math.isfinite(value) for value in (*candidate.size, *candidate.translation)):
        return CandidateTop(
            success=False,
            diagnostic="candidate size or pose is non-finite",
            rectangle=None,
            **base,
        )
    if (
        candidate.size[0] <= minimum_edge_m
        or candidate.size[1] <= minimum_edge_m
        or candidate.size[2] < 0.0
    ):
        return CandidateTop(
            success=False,
            diagnostic="candidate has a degenerate size",
            rectangle=None,
            **base,
        )
    if len(candidate.hull) < 3 or not all(
        math.isfinite(value) for point in candidate.hull for value in point
    ):
        return CandidateTop(
            success=False,
            diagnostic="candidate hull is empty, undersized, or non-finite",
            rectangle=None,
            **base,
        )

    half_x = candidate.size[0] * 0.5
    half_y = candidate.size[1] * 0.5
    top_z = candidate.size[2] * 0.5
    local_points = (
        (-half_x, -half_y, top_z),
        (half_x, -half_y, top_z),
        (half_x, half_y, top_z),
        (-half_x, half_y, top_z),
    )
    try:
        points = tuple(
            _add(
                _rotate_by_quaternion(point, candidate.quaternion_xyzw),
                candidate.translation,
            )
            for point in local_points
        )
    except DiagnosticError as error:
        return CandidateTop(
            success=False,
            diagnostic=str(error),
            rectangle=None,
            **base,
        )

    projected_area = _signed_xy_area(points)
    if (
        not math.isfinite(projected_area)
        or abs(projected_area) <= minimum_edge_m * minimum_edge_m
    ):
        return CandidateTop(
            success=False,
            diagnostic="candidate top face is degenerate in XY",
            rectangle=None,
            **base,
        )
    if projected_area > 0.0:
        points = (points[0], points[3], points[2], points[1])
    return CandidateTop(
        success=True,
        diagnostic="candidate top rectangle recovered in clockwise XY order",
        rectangle=_rectangle(points),
        **base,
    )


def _normalize_xy(vector: tuple[float, float], minimum_norm: float):
    norm = math.hypot(vector[0], vector[1])
    if not math.isfinite(norm) or norm <= minimum_norm:
        return None
    return (vector[0] / norm, vector[1] / norm)


def _solve_3x3(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
    minimum_pivot: float = 1.0e-12,
) -> Optional[tuple[float, float, float]]:
    augmented = [
        [float(matrix[row][column]) for column in range(3)] + [float(vector[row])]
        for row in range(3)
    ]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) <= minimum_pivot:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        pivot_value = augmented[column][column]
        for value_index in range(column, 4):
            augmented[column][value_index] /= pivot_value
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            for value_index in range(column, 4):
                augmented[row][value_index] -= (
                    factor * augmented[column][value_index]
                )
    solution = tuple(augmented[row][3] for row in range(3))
    if not all(math.isfinite(value) for value in solution):
        return None
    return solution  # type: ignore[return-value]


def _estimate_stair_pose(
    candidates: Sequence[CandidateTop],
    minimum_edge_m: float,
) -> dict[str, Any]:
    eligible = []
    for candidate in candidates:
        if not candidate.success or candidate.rectangle is None:
            continue
        points = candidate.rectangle.points
        first_length = _distance_xy(points[0], points[1])
        second_length = _distance_xy(points[1], points[2])
        if (
            max(first_length, second_length)
            > FACTORY_STAIR_POSE_MINIMUM_LONG_EDGE_M
            and min(first_length, second_length)
            > FACTORY_STAIR_POSE_MINIMUM_SHORT_EDGE_M
        ):
            eligible.append(candidate)
    result: dict[str, Any] = {
        "success": False,
        "eligible_quadrangle_count": len(eligible),
        "direction_xy": None,
        "diagnostic": (
            "factory stair pose requires more than one eligible quadrangle"
        ),
    }
    if len(eligible) <= 1:
        return result

    normal = [[0.0] * 3 for _ in range(3)]
    right_hand_side = [0.0] * 3
    for candidate in eligible:
        assert candidate.rectangle is not None
        for point in candidate.rectangle.points:
            for row in range(3):
                right_hand_side[row] -= point[row]
                for column in range(3):
                    normal[row][column] += point[row] * point[column]
    plane = _solve_3x3(normal, right_hand_side)
    if plane is None:
        result["diagnostic"] = "stair pose least-squares solution is singular"
        return result
    if plane[2] < 0.0:
        plane = tuple(-value for value in plane)
    direction = _normalize_xy(
        (-plane[0] * plane[2], -plane[1] * plane[2]),
        minimum_edge_m,
    )
    if direction is None:
        result["diagnostic"] = "stair pose horizontal gradient is degenerate"
        return result
    result.update(
        {
            "success": True,
            "direction_xy": list(direction),
            "diagnostic": "factory least-squares stair pose recovered",
        }
    )
    return result


def _correct_candidate_top(
    candidate: CandidateTop,
    direction: tuple[float, float],
) -> CandidateTop:
    assert candidate.success and candidate.rectangle is not None
    lateral = (-direction[1], direction[0])
    source_points = candidate.contained_points or candidate.hull
    along = [
        direction[0] * point[0] + direction[1] * point[1]
        for point in source_points
    ]
    across = [
        lateral[0] * point[0] + lateral[1] * point[1]
        for point in source_points
    ]
    mean_z = math.fsum(point[2] for point in candidate.rectangle.points) * 0.25

    def world(along_value: float, across_value: float) -> Point:
        return (
            direction[0] * along_value + lateral[0] * across_value,
            direction[1] * along_value + lateral[1] * across_value,
            mean_z,
        )

    points = (
        world(min(along), min(across)),
        world(min(along), max(across)),
        world(max(along), max(across)),
        world(max(along), min(across)),
    )
    if _signed_xy_area(points) > 0.0:
        points = (points[0], points[3], points[2], points[1])
    return CandidateTop(
        candidate_index=candidate.candidate_index,
        success=True,
        diagnostic=(
            "correctQuadPose mirror using recorded candidate hull as contained-cloud proxy"
        ),
        rectangle=_rectangle(points),
        hull_point_count=candidate.hull_point_count,
        hull_area_3d_m2=candidate.hull_area_3d_m2,
        size=candidate.size,
        translation=candidate.translation,
        hull=candidate.hull,
        contained_points=candidate.contained_points,
        contained_points_are_recorded=candidate.contained_points_are_recorded,
    )


def correct_candidate_top_rectangles(
    candidates: Sequence[CandidateTop],
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    valid = [
        candidate
        for candidate in candidates
        if candidate.success
        and candidate.rectangle is not None
        and len(candidate.hull) >= 3
    ]
    estimate = _estimate_stair_pose(valid, minimum_edge_m)
    report: dict[str, Any] = {
        "success": False,
        "input_source": (
            "recorded_candidate_contained_points"
            if valid and all(
                candidate.contained_points_are_recorded for candidate in valid
            )
            else "recorded_candidate_hull_proxy_not_factory_mPointClouds"
        ),
        "input_candidate_count": len(candidates),
        "valid_seed_count": len(valid),
        "eligible_stair_pose_count": estimate["eligible_quadrangle_count"],
        "consensus_direction_count": 0,
        "initial_stair_direction_xy": estimate["direction_xy"],
        "corrected_stair_direction_xy": None,
        "diagnostic": estimate["diagnostic"],
    }
    if not estimate["success"]:
        return list(candidates), report
    if len(valid) < FACTORY_CORRECT_MINIMUM_SIZE:
        report["diagnostic"] = (
            "factory pose correction minimum quadrangle count was not met"
        )
        return list(candidates), report

    stair_direction = tuple(estimate["direction_xy"])
    oriented = []
    for candidate in valid:
        assert candidate.rectangle is not None
        points = candidate.rectangle.points
        first = _normalize_xy(
            (points[0][0] - points[1][0], points[0][1] - points[1][1]),
            minimum_edge_m,
        )
        second = _normalize_xy(
            (points[1][0] - points[2][0], points[1][1] - points[2][1]),
            minimum_edge_m,
        )
        if first is None or second is None:
            oriented.append((0.0, 0.0))
            continue
        first_dot = first[0] * stair_direction[0] + first[1] * stair_direction[1]
        second_dot = (
            second[0] * stair_direction[0] + second[1] * stair_direction[1]
        )
        selected = first if abs(first_dot) >= abs(second_dot) else second
        if selected[0] * stair_direction[0] + selected[1] * stair_direction[1] < 0:
            selected = (-selected[0], -selected[1])
        oriented.append(selected)

    average = (
        math.fsum(direction[0] for direction in oriented) / len(oriented),
        math.fsum(direction[1] for direction in oriented) / len(oriented),
    )
    consensus_members = [
        direction
        for direction in oriented
        if average[0] * direction[0] + average[1] * direction[1]
        > FACTORY_POSE_CONSENSUS_COSINE
    ]
    report["consensus_direction_count"] = len(consensus_members)
    if len(consensus_members) <= 1:
        report["diagnostic"] = (
            "factory pose correction direction consensus was not met"
        )
        return list(candidates), report
    consensus = _normalize_xy(
        (
            math.fsum(direction[0] for direction in consensus_members)
            / len(consensus_members),
            math.fsum(direction[1] for direction in consensus_members)
            / len(consensus_members),
        ),
        minimum_edge_m,
    )
    if consensus is None:
        report["diagnostic"] = "factory pose correction consensus is degenerate"
        return list(candidates), report

    corrected_by_index = {
        candidate.candidate_index: _correct_candidate_top(candidate, consensus)
        for candidate in valid
    }
    corrected = [
        corrected_by_index.get(candidate.candidate_index, candidate)
        for candidate in candidates
    ]
    report.update(
        {
            "success": True,
            "corrected_stair_direction_xy": list(consensus),
            "diagnostic": (
                "computeStairPose and correctQuadPose mirrored with hull proxy"
            ),
        }
    )
    return corrected, report


def _candidate_direction_aligned_with_stair(
    candidate: CandidateTop,
    stair_direction: tuple[float, float],
    minimum_edge_m: float,
) -> Optional[tuple[float, float]]:
    if not candidate.success or candidate.rectangle is None:
        return None
    points = candidate.rectangle.points
    first = _normalize_xy(
        (points[0][0] - points[1][0], points[0][1] - points[1][1]),
        minimum_edge_m,
    )
    second = _normalize_xy(
        (points[1][0] - points[2][0], points[1][1] - points[2][1]),
        minimum_edge_m,
    )
    if first is None or second is None:
        return None
    selected = (
        first
        if abs(first[0] * stair_direction[0] + first[1] * stair_direction[1])
        >= abs(
            second[0] * stair_direction[0]
            + second[1] * stair_direction[1]
        )
        else second
    )
    if selected[0] * stair_direction[0] + selected[1] * stair_direction[1] < 0:
        selected = (-selected[0], -selected[1])
    return selected


def _candidate_with_geometry(
    candidate: CandidateTop,
    points: Sequence[Point],
    working_points: Sequence[Point],
    diagnostic: str,
    *,
    hull: Optional[Sequence[Point]] = None,
    contained_points_are_recorded: Optional[bool] = None,
) -> CandidateTop:
    fixed_points = tuple(points)
    if _signed_xy_area(fixed_points) > 0.0:
        fixed_points = (
            fixed_points[0],
            fixed_points[3],
            fixed_points[2],
            fixed_points[1],
        )
    return CandidateTop(
        candidate_index=candidate.candidate_index,
        success=True,
        diagnostic=diagnostic,
        rectangle=_rectangle(fixed_points),
        hull_point_count=candidate.hull_point_count,
        hull_area_3d_m2=candidate.hull_area_3d_m2,
        size=candidate.size,
        translation=candidate.translation,
        hull=tuple(candidate.hull if hull is None else hull),
        contained_points=tuple(working_points),
        contained_points_are_recorded=(
            candidate.contained_points_are_recorded
            if contained_points_are_recorded is None
            else contained_points_are_recorded
        ),
    )


def cut_candidate_tops_by_x(
    candidates: Sequence[CandidateTop],
    stair_direction: Sequence[float],
    *,
    target_candidate_indices: Optional[set[int]] = None,
    minimum_remaining_depth_m: float = FACTORY_CUT_MINIMUM_REMAINING_DEPTH_M,
    step_m: float = FACTORY_CUT_STEP_M,
    removed_point_ratio: float = FACTORY_CUT_REMOVED_POINT_RATIO,
    maximum_removed_points: int = FACTORY_CUT_MAXIMUM_REMOVED_POINTS,
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    report: dict[str, Any] = {
        "success": False,
        "input_candidate_count": len(candidates),
        "processed_candidate_count": 0,
        "changed_candidate_count": 0,
        "changed_candidate_indices": [],
        "input_source": (
            "recorded_candidate_contained_points"
            if candidates
            and all(
                not candidate.success or candidate.contained_points_are_recorded
                for candidate in candidates
            )
            else "recorded_candidate_hull_proxy_not_factory_mPointClouds"
        ),
        "diagnostic": "cutByX has not run",
    }
    if (
        not math.isfinite(minimum_remaining_depth_m)
        or minimum_remaining_depth_m <= 0.0
        or not math.isfinite(step_m)
        or step_m <= 0.0
        or not math.isfinite(removed_point_ratio)
        or removed_point_ratio <= 0.0
        or maximum_removed_points <= 0
    ):
        report["diagnostic"] = "invalid cutByX configuration"
        return list(candidates), report
    normalized_stair = _normalize_xy(
        (float(stair_direction[0]), float(stair_direction[1])),
        minimum_edge_m,
    )
    if normalized_stair is None:
        report["diagnostic"] = "cutByX stair direction is degenerate"
        return list(candidates), report

    output: list[CandidateTop] = []
    for candidate in candidates:
        if (
            not candidate.success
            or candidate.rectangle is None
            or (
                target_candidate_indices is not None
                and candidate.candidate_index not in target_candidate_indices
            )
        ):
            output.append(candidate)
            continue
        report["processed_candidate_count"] += 1
        source_points = list(candidate.contained_points or candidate.hull)
        if not source_points:
            output.append(candidate)
            continue
        points = list(candidate.rectangle.points)
        first_direction = _normalize_xy(
            (
                points[0][0] - points[1][0],
                points[0][1] - points[1][1],
            ),
            minimum_edge_m,
        )
        second_direction = _normalize_xy(
            (
                points[1][0] - points[2][0],
                points[1][1] - points[2][1],
            ),
            minimum_edge_m,
        )
        if first_direction is None or second_direction is None:
            output.append(candidate)
            continue
        if abs(
            first_direction[0] * normalized_stair[0]
            + first_direction[1] * normalized_stair[1]
        ) >= abs(
            second_direction[0] * normalized_stair[0]
            + second_direction[1] * normalized_stair[1]
        ):
            points = [points[1], points[2], points[3], points[0]]

        keep = [True] * len(source_points)
        accepted_any_cut = False
        for _ in range(2):
            first_point = list(points[0])
            second_point = list(points[1])
            third_point = points[2]
            fourth_point = points[3]
            boundary = (
                second_point[0] - first_point[0],
                second_point[1] - first_point[1],
            )
            boundary_length = math.hypot(*boundary)
            allowed_removed = min(
                math.floor(
                    boundary_length
                    / minimum_remaining_depth_m
                    * removed_point_ratio
                ),
                maximum_removed_points,
            )
            depth_delta = (
                fourth_point[0] - first_point[0],
                fourth_point[1] - first_point[1],
                fourth_point[2] - first_point[2],
            )
            remaining = math.hypot(depth_delta[0], depth_delta[1])
            if remaining > minimum_remaining_depth_m:
                step = tuple(
                    value * step_m / remaining for value in depth_delta
                )
                while remaining > minimum_remaining_depth_m:
                    for coordinate in range(3):
                        first_point[coordinate] += step[coordinate]
                        second_point[coordinate] += step[coordinate]
                    remaining -= step_m
                    removed_this_step: list[int] = []
                    for point_index, point in enumerate(source_points):
                        if not keep[point_index]:
                            continue
                        delta_x = point[0] - first_point[0]
                        delta_y = point[1] - first_point[1]
                        if (
                            boundary[1] * delta_x
                            - boundary[0] * delta_y
                            > 0.0
                        ):
                            keep[point_index] = False
                            removed_this_step.append(point_index)
                        if len(removed_this_step) > allowed_removed:
                            break
                    if len(removed_this_step) > allowed_removed:
                        for point_index in removed_this_step:
                            keep[point_index] = True
                        for coordinate in range(3):
                            first_point[coordinate] -= step[coordinate]
                            second_point[coordinate] -= step[coordinate]
                        remaining += step_m
                        break
                    accepted_any_cut = True
            points = [
                third_point,
                fourth_point,
                tuple(first_point),
                tuple(second_point),
            ]

        if not accepted_any_cut:
            output.append(candidate)
            continue
        replacement = next(
            (
                source_points[index]
                for index in range(len(source_points) - 1, -1, -1)
                if keep[index]
            ),
            (0.0, 0.0, 0.0),
        )
        working_points = tuple(
            point if keep[index] else replacement
            for index, point in enumerate(source_points)
        )
        output.append(
            _candidate_with_geometry(
                candidate,
                points,
                working_points,
                "factory cutByX two-sided 3 cm scan mirror",
            )
        )
        report["changed_candidate_indices"].append(candidate.candidate_index)
        report["changed_candidate_count"] += 1

    report.update(
        {
            "success": True,
            "diagnostic": "factory cutByX two-sided 3 cm scan completed",
        }
    )
    return output, report


def _sample_boundary(
    rectangle: Rectangle,
    sample_step_m: float = FACTORY_BOUNDARY_SAMPLE_STEP_M,
    sample_count_epsilon: float = FACTORY_BOUNDARY_SAMPLE_COUNT_EPSILON,
) -> list[tuple[float, float]]:
    samples = []
    for index, start in enumerate(rectangle.points):
        end = rectangle.points[(index + 1) % 4]
        delta = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(*delta)
        sample_count = int(length / sample_step_m + sample_count_epsilon)
        if sample_count <= 0 or length <= 0.0:
            continue
        step = (
            delta[0] / length * sample_step_m,
            delta[1] / length * sample_step_m,
        )
        samples.extend(
            (
                start[0] + sample_index * step[0],
                start[1] + sample_index * step[1],
            )
            for sample_index in range(sample_count)
        )
    return samples


def _factory_pcl_boundary_intrudes(
    rectangle: Rectangle,
    sampled_boundary: Sequence[tuple[float, float]],
) -> bool:
    """Mirror configIntrusion's float32 PCL transform and PassThrough gate."""

    first = rectangle.points[0]
    origin = rectangle.points[1]
    second = rectangle.points[2]
    first_axis = (first[0] - origin[0], first[1] - origin[1])
    second_axis = (second[0] - origin[0], second[1] - origin[1])
    first_length = math.hypot(*first_axis)
    second_length = math.hypot(*second_axis)
    if first_length <= 0.0 or second_length <= 0.0:
        return False

    angle = math.atan2(first_axis[1], first_axis[0])
    cosine = math.cos(angle)
    sine = math.sin(angle)

    # The factory transforms the rectangle origin in double precision, then
    # converts the PassThrough limits to float. Boundary samples first become
    # PointXYZ float values and use the float32 transform matrix.
    minimum_x = _float32(
        cosine * origin[0] + sine * origin[1]
    )
    maximum_x = _float32(
        cosine * origin[0] + sine * origin[1] + first_length
    )
    minimum_y = _float32(
        -sine * origin[0] + cosine * origin[1]
    )
    maximum_y = _float32(
        -sine * origin[0] + cosine * origin[1] + second_length
    )
    matrix_cosine = _float32(cosine)
    matrix_sine = _float32(sine)

    for point in sampled_boundary:
        point_x = _float32(point[0])
        point_y = _float32(point[1])
        local_x = _float32(
            _float32(matrix_cosine * point_x) +
            matrix_sine * point_y
        )
        local_y = _float32(
            _float32(-matrix_sine * point_x) +
            matrix_cosine * point_y
        )
        if (
            minimum_x <= local_x <= maximum_x and
            minimum_y <= local_y <= maximum_y
        ):
            return True
    return False


def compute_candidate_distance_vector_xy(
    first: CandidateTop,
    second: CandidateTop,
) -> tuple[float, float]:
    if first.rectangle is None or second.rectangle is None:
        return (100.0, 0.0)
    first_boundary = _sample_boundary(first.rectangle)
    second_boundary = _sample_boundary(second.rectangle)
    if (
        _factory_pcl_boundary_intrudes(
            first.rectangle, second_boundary
        ) or
        _factory_pcl_boundary_intrudes(
            second.rectangle, first_boundary
        )
    ):
        return (0.0, 0.0)
    best = (100.0, 0.0)
    best_squared = 10000.0
    for first_point in first_boundary:
        for second_point in second_boundary:
            candidate = (
                first_point[0] - second_point[0],
                first_point[1] - second_point[1],
            )
            squared = candidate[0] ** 2 + candidate[1] ** 2
            if squared < best_squared:
                best_squared = squared
                best = candidate
    return best


def _aligned_candidate_from_points(
    candidate: CandidateTop,
    source_points: Sequence[Point],
    direction: tuple[float, float],
    diagnostic: str,
    *,
    hull: Optional[Sequence[Point]] = None,
    contained_points_are_recorded: Optional[bool] = None,
) -> CandidateTop:
    direction_norm = math.hypot(*direction)
    unit_direction = (
        direction[0] / direction_norm,
        direction[1] / direction_norm,
    )
    lateral = (-unit_direction[1], unit_direction[0])
    along = [
        unit_direction[0] * point[0] + unit_direction[1] * point[1]
        for point in source_points
    ]
    across = [
        lateral[0] * point[0] + lateral[1] * point[1]
        for point in source_points
    ]
    mean_z = math.fsum(point[2] for point in source_points) / len(source_points)

    def world(along_value: float, across_value: float) -> Point:
        return (
            unit_direction[0] * along_value + lateral[0] * across_value,
            unit_direction[1] * along_value + lateral[1] * across_value,
            mean_z,
        )

    return _candidate_with_geometry(
        candidate,
        (
            world(min(along), min(across)),
            world(min(along), max(across)),
            world(max(along), max(across)),
            world(max(along), min(across)),
        ),
        source_points,
        diagnostic,
        hull=hull,
        contained_points_are_recorded=contained_points_are_recorded,
    )


def merge_candidate_tops_at_same_stair(
    candidates: Sequence[CandidateTop],
    stair_direction: Sequence[float],
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    normalized_stair = _normalize_xy(
        (float(stair_direction[0]), float(stair_direction[1])),
        minimum_edge_m,
    )
    report: dict[str, Any] = {
        "success": False,
        "input_candidate_count": sum(candidate.success for candidate in candidates),
        "output_candidate_count": 0,
        "merged_pair_count": 0,
        "post_merge_cut_changed_count": 0,
        "merged_source_candidate_indices": [],
        "diagnostic": "same-stair merge has not run",
    }
    if normalized_stair is None:
        report["diagnostic"] = "same-stair merge direction is degenerate"
        return list(candidates), report

    records = []
    invalid = []
    for candidate in candidates:
        direction = _candidate_direction_aligned_with_stair(
            candidate, normalized_stair, minimum_edge_m
        )
        if direction is None or candidate.rectangle is None:
            invalid.append(candidate)
            continue
        records.append(
            {
                "candidate": candidate,
                "direction": direction,
                "mean_z": candidate.rectangle.center[2],
                "merge_count": 1,
                "source_indices": [candidate.candidate_index],
                "active": True,
            }
        )

    for first_index, first_record in enumerate(records):
        if not first_record["active"]:
            continue
        for second_index in range(first_index + 1, len(records)):
            second_record = records[second_index]
            if not second_record["active"]:
                continue
            if (
                abs(first_record["mean_z"] - second_record["mean_z"])
                > FACTORY_MERGE_MAXIMUM_HEIGHT_DIFFERENCE_M
            ):
                continue
            if (
                abs(
                    first_record["direction"][0]
                    * second_record["direction"][0]
                    + first_record["direction"][1]
                    * second_record["direction"][1]
                )
                < FACTORY_MERGE_DIRECTION_COSINE
            ):
                continue
            distance = compute_candidate_distance_vector_xy(
                first_record["candidate"], second_record["candidate"]
            )
            if math.hypot(*distance) > FACTORY_MERGE_MAXIMUM_DISTANCE_M:
                continue
            if (
                abs(
                    distance[0] * normalized_stair[0]
                    + distance[1] * normalized_stair[1]
                )
                > FACTORY_MERGE_MAXIMUM_ALONG_STAIR_DISTANCE_M
            ):
                continue
            first_candidate = first_record["candidate"]
            second_candidate = second_record["candidate"]
            source_points = tuple(
                first_candidate.contained_points
                or first_candidate.hull
            ) + tuple(
                second_candidate.contained_points
                or second_candidate.hull
            )
            first_count = first_record["merge_count"]
            second_count = second_record["merge_count"]
            combined_count = first_count + second_count
            combined_direction = (
                (
                    first_record["direction"][0] * first_count
                    + second_record["direction"][0] * second_count
                )
                / combined_count,
                (
                    first_record["direction"][1] * first_count
                    + second_record["direction"][1] * second_count
                )
                / combined_count,
            )
            if (
                not all(math.isfinite(value) for value in combined_direction)
                or math.hypot(*combined_direction) <= minimum_edge_m
            ):
                continue
            first_record["candidate"] = _aligned_candidate_from_points(
                first_candidate,
                source_points,
                combined_direction,
                "factory same-stair merged quadrangle mirror",
                hull=first_candidate.hull + second_candidate.hull,
                contained_points_are_recorded=(
                    first_candidate.contained_points_are_recorded
                    and second_candidate.contained_points_are_recorded
                ),
            )
            first_record["direction"] = combined_direction
            first_record["merge_count"] = combined_count
            first_record["source_indices"].extend(
                second_record["source_indices"]
            )
            second_record["active"] = False
            report["merged_pair_count"] += 1

    active_candidates = [
        record["candidate"] for record in records if record["active"]
    ]
    merged_indices = {
        record["candidate"].candidate_index
        for record in records
        if record["active"] and record["merge_count"] > 1
    }
    report["merged_source_candidate_indices"] = [
        record["source_indices"]
        for record in records
        if record["active"] and record["merge_count"] > 1
    ]
    cut_candidates, cut_report = cut_candidate_tops_by_x(
        active_candidates,
        normalized_stair,
        target_candidate_indices=merged_indices,
    )
    if not cut_report["success"]:
        report["diagnostic"] = (
            f"post-merge cutByX failed: {cut_report['diagnostic']}"
        )
        return list(candidates), report
    report.update(
        {
            "success": True,
            "output_candidate_count": len(cut_candidates),
            "post_merge_cut_changed_count": cut_report[
                "changed_candidate_count"
            ],
            "post_merge_cut": cut_report,
            "diagnostic": (
                "factory same-stair merge and post-merge cutByX completed"
            ),
        }
    )
    return cut_candidates + invalid, report


def _side_distance_outlier_indices(
    candidates: Sequence[CandidateTop],
    axis: tuple[float, float],
    threshold_m: float,
) -> list[int]:
    indices = []
    for vector_index in range(len(candidates) - 1, -1, -1):
        candidate = candidates[vector_index]
        if not candidate.success or candidate.rectangle is None:
            continue
        sides = []
        distances = []
        for point in candidate.rectangle.points:
            projection = axis[0] * point[0] + axis[1] * point[1]
            squared_distance = max(
                0.0,
                point[0] * point[0]
                + point[1] * point[1]
                - projection * projection,
            )
            distances.append(math.sqrt(squared_distance))
            sides.append(axis[1] * point[0] - axis[0] * point[1] > 0.0)
        if all(distance > threshold_m for distance in distances) and all(
            side == sides[0] for side in sides[1:]
        ):
            indices.append(vector_index)
    return indices


def ignore_unnecessary_candidate_tops(
    candidates: Sequence[CandidateTop],
    stair_direction: Sequence[float],
    side_distance_threshold_m: float = FACTORY_SIDE_DISTANCE_THRESHOLD_M,
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    report: dict[str, Any] = {
        "success": False,
        "side_distance_threshold_m": side_distance_threshold_m,
        "selected_stair_direction_xy": None,
        "original_axis_removal_count": 0,
        "perpendicular_axis_removal_count": 0,
        "removed_vector_indices": [],
        "removed_candidate_indices": [],
        "diagnostic": "side-distance filter has not run",
    }
    if (
        not math.isfinite(side_distance_threshold_m)
        or side_distance_threshold_m <= 0.0
    ):
        report["diagnostic"] = (
            "side-distance thresholds must be finite and positive"
        )
        return list(candidates), report
    original = _normalize_xy(
        (float(stair_direction[0]), float(stair_direction[1])),
        minimum_edge_m,
    )
    if original is None:
        report["diagnostic"] = "side-distance stair direction is degenerate"
        return list(candidates), report
    perpendicular = (original[1], -original[0])
    original_removals = _side_distance_outlier_indices(
        candidates, original, side_distance_threshold_m
    )
    perpendicular_removals = _side_distance_outlier_indices(
        candidates, perpendicular, side_distance_threshold_m
    )
    use_perpendicular = len(original_removals) > len(perpendicular_removals)
    selected = perpendicular if use_perpendicular else original
    removals = perpendicular_removals if use_perpendicular else original_removals
    removal_set = set(removals)
    filtered = [
        candidate
        for vector_index, candidate in enumerate(candidates)
        if vector_index not in removal_set
    ]
    report.update(
        {
            "success": True,
            "selected_stair_direction_xy": list(selected),
            "original_axis_removal_count": len(original_removals),
            "perpendicular_axis_removal_count": len(perpendicular_removals),
            "removed_vector_indices": removals,
            "removed_candidate_indices": [
                candidates[index].candidate_index for index in removals
            ],
            "diagnostic": (
                "factory side-distance filter selected the perpendicular stair axis"
                if use_perpendicular
                else "factory side-distance filter retained the original stair axis"
            ),
        }
    )
    return filtered, report


def _sample_intrusion_boundary(
    candidate: CandidateTop,
    grid_resolution_m: float,
) -> tuple[tuple[float, float], ...]:
    assert candidate.rectangle is not None
    samples = []
    points = candidate.rectangle.points
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        delta = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(*delta)
        if not math.isfinite(length) or length <= 0.0:
            continue
        sample_count = int(length / grid_resolution_m + 1.0)
        step = (
            delta[0] / length * grid_resolution_m,
            delta[1] / length * grid_resolution_m,
        )
        samples.extend(
            (
                start[0] + sample_index * step[0],
                start[1] + sample_index * step[1],
            )
            for sample_index in range(sample_count)
        )
    return tuple(samples)


def _sampled_boundary_enters_candidate(
    candidate: CandidateTop,
    sampled_boundary: Sequence[tuple[float, float]],
    minimum_edge_m: float,
) -> bool:
    assert candidate.rectangle is not None
    points = candidate.rectangle.points
    origin = points[1]
    first_axis = (points[0][0] - origin[0], points[0][1] - origin[1])
    second_axis = (points[2][0] - origin[0], points[2][1] - origin[1])
    first_length = math.hypot(*first_axis)
    second_length = math.hypot(*second_axis)
    if (
        not math.isfinite(first_length)
        or not math.isfinite(second_length)
        or first_length <= minimum_edge_m
        or second_length <= minimum_edge_m
    ):
        return False
    first_unit = (first_axis[0] / first_length, first_axis[1] / first_length)
    second_unit = (
        second_axis[0] / second_length,
        second_axis[1] / second_length,
    )
    for point in sampled_boundary:
        local = (point[0] - origin[0], point[1] - origin[1])
        first_coordinate = (
            first_unit[0] * local[0] + first_unit[1] * local[1]
        )
        second_coordinate = (
            second_unit[0] * local[0] + second_unit[1] * local[1]
        )
        if (
            0.0 <= first_coordinate <= first_length
            and 0.0 <= second_coordinate <= second_length
        ):
            return True
    return False


def classify_candidate_intrusions(
    candidates: Sequence[CandidateTop],
    grid_resolution_m: float,
    maximum_primary_short_edge_m: float = (
        FACTORY_SORT_MAXIMUM_PRIMARY_SHORT_EDGE_M
    ),
    minimum_primary_long_edge_m: float = (
        FACTORY_SORT_MINIMUM_PRIMARY_LONG_EDGE_M
    ),
    minimum_edge_m: float = 1.0e-6,
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "success": False,
        "grid_resolution_m": grid_resolution_m,
        "maximum_primary_short_edge_m": maximum_primary_short_edge_m,
        "minimum_primary_long_edge_m": minimum_primary_long_edge_m,
        "input_candidate_count": len(candidates),
        "valid_candidate_count": 0,
        "invalid_vector_indices": [],
        "sorted_vector_indices": [],
        "sorted_candidate_indices": [],
        "non_intruding_vector_indices": [],
        "intruded_short_edge_vector_indices": [],
        "intruded_long_edge_vector_indices": [],
        "diagnostic": "quadrangle intrusion classification has not run",
    }
    thresholds = (
        grid_resolution_m,
        maximum_primary_short_edge_m,
        minimum_primary_long_edge_m,
        minimum_edge_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in thresholds):
        report["diagnostic"] = (
            "intrusion classification thresholds must be finite and positive"
        )
        return report

    records = []
    for vector_index, candidate in enumerate(candidates):
        if not candidate.success or candidate.rectangle is None:
            report["invalid_vector_indices"].append(vector_index)
            continue
        points = candidate.rectangle.points
        first_length = math.hypot(
            points[0][0] - points[1][0],
            points[0][1] - points[1][1],
        )
        second_length = math.hypot(
            points[1][0] - points[2][0],
            points[1][1] - points[2][1],
        )
        if (
            not math.isfinite(first_length)
            or not math.isfinite(second_length)
            or first_length <= minimum_edge_m
            or second_length <= minimum_edge_m
        ):
            report["invalid_vector_indices"].append(vector_index)
            continue
        records.append(
            {
                "vector_index": vector_index,
                "candidate_index": candidate.candidate_index,
                "short_edge_m": min(first_length, second_length),
                "long_edge_m": max(first_length, second_length),
                "sampled_boundary": _sample_intrusion_boundary(
                    candidate, grid_resolution_m
                ),
            }
        )
    report["valid_candidate_count"] = len(records)

    records.sort(key=lambda record: record["short_edge_m"])
    primary = [
        record
        for record in records
        if record["short_edge_m"] <= maximum_primary_short_edge_m
        and record["long_edge_m"] >= minimum_primary_long_edge_m
    ]
    deferred = [
        record
        for record in records
        if record["short_edge_m"] > maximum_primary_short_edge_m
        or record["long_edge_m"] < minimum_primary_long_edge_m
    ]
    ordered = primary + deferred
    report["sorted_vector_indices"] = [
        record["vector_index"] for record in ordered
    ]
    report["sorted_candidate_indices"] = [
        record["candidate_index"] for record in ordered
    ]
    if ordered:
        report["non_intruding_vector_indices"].append(
            ordered[0]["vector_index"]
        )

    for current_position in range(1, len(ordered)):
        current = ordered[current_position]
        intruded = any(
            _sampled_boundary_enters_candidate(
                candidates[current["vector_index"]],
                ordered[previous_position]["sampled_boundary"],
                minimum_edge_m,
            )
            for previous_position in range(current_position)
        )
        if not intruded:
            report["non_intruding_vector_indices"].append(
                current["vector_index"]
            )
        elif current["short_edge_m"] > maximum_primary_short_edge_m:
            report["intruded_long_edge_vector_indices"].append(
                current["vector_index"]
            )
        else:
            report["intruded_short_edge_vector_indices"].append(
                current["vector_index"]
            )

    report["success"] = True
    report["diagnostic"] = (
        "factory pcl2vectors, sortVectors, and configIntrusions mirrored"
    )
    return report


def compute_candidate_reference_distance_m(
    first: CandidateTop,
    second: CandidateTop,
) -> float:
    if first.rectangle is None or second.rectangle is None:
        return 100.0
    first_boundary = _sample_boundary(
        first.rectangle,
        FACTORY_REFERENCE_DISTANCE_SAMPLE_STEP_M,
        FACTORY_REFERENCE_DISTANCE_SAMPLE_COUNT_EPSILON,
    )
    second_boundary = _sample_boundary(
        second.rectangle,
        FACTORY_REFERENCE_DISTANCE_SAMPLE_STEP_M,
        FACTORY_REFERENCE_DISTANCE_SAMPLE_COUNT_EPSILON,
    )
    if not first_boundary or not second_boundary:
        return 100.0
    minimum_squared_distance = 10000.0
    for first_point in first_boundary:
        for second_point in second_boundary:
            minimum_squared_distance = min(
                minimum_squared_distance,
                (first_point[0] - second_point[0]) ** 2
                + (first_point[1] - second_point[1]) ** 2,
            )
    return math.sqrt(minimum_squared_distance)


def select_candidate_reference(
    candidates: Sequence[CandidateTop],
    reference_vector_indices: Sequence[int],
    target_vector_index: int,
    grid_resolution_m: float,
    grid_map_position_xy: tuple[float, float],
    fallback: bool,
    maximum_candidate_distance_m: float = (
        FACTORY_REFERENCE_MAXIMUM_CANDIDATE_DISTANCE_M
    ),
    near_distance_m: float = FACTORY_REFERENCE_NEAR_DISTANCE_M,
    minimum_edge_m: float = 1.0e-6,
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "success": False,
        "selected_reference_vector_index": -1,
        "target_added_to_reference_indices": False,
        "deferred_for_fallback": False,
        "updated_reference_vector_indices": list(reference_vector_indices),
        "constraint_points_xyz": [],
        "diagnostic": "reference selection has not run",
    }
    thresholds = (
        grid_resolution_m,
        maximum_candidate_distance_m,
        near_distance_m,
        minimum_edge_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in thresholds):
        report["diagnostic"] = (
            "reference selection thresholds must be finite and positive"
        )
        return report
    if (
        target_vector_index < 0
        or target_vector_index >= len(candidates)
        or any(index < 0 or index >= len(candidates)
               for index in reference_vector_indices)
    ):
        report["diagnostic"] = "reference selection index is outside the vector"
        return report
    if not all(math.isfinite(value) for value in grid_map_position_xy):
        report["diagnostic"] = "GridMap position must be finite"
        return report
    if any(
        not candidate.success or candidate.rectangle is None
        for candidate in candidates
    ):
        report["diagnostic"] = "reference selection requires valid rectangles"
        return report

    boundaries = [
        _sample_intrusion_boundary(candidate, grid_resolution_m)
        for candidate in candidates
    ]
    if any(not boundary for boundary in boundaries):
        report["diagnostic"] = "reference rectangle has an empty boundary"
        return report
    heights = [candidate.rectangle.center[2] for candidate in candidates]
    target_height = heights[target_vector_index]
    lower = sorted(
        (
            (index, heights[index])
            for index in reference_vector_indices
            if heights[index] < target_height
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    upper_or_equal = sorted(
        (
            (index, heights[index])
            for index in reference_vector_indices
            if heights[index] >= target_height
        ),
        key=lambda item: item[1],
    )

    def intrudes(rectangle_index: int, boundary_index: int) -> bool:
        return _sampled_boundary_enters_candidate(
            candidates[rectangle_index],
            boundaries[boundary_index],
            minimum_edge_m,
        )

    def related(reference_index: int) -> bool:
        return (
            intrudes(target_vector_index, reference_index)
            or intrudes(reference_index, target_vector_index)
            or compute_candidate_reference_distance_m(
                candidates[reference_index],
                candidates[target_vector_index],
            )
            <= maximum_candidate_distance_m
        )

    def first_related(items: Sequence[tuple[int, float]]) -> int:
        return next((index for index, _ in items if related(index)), -1)

    lower_reference = first_related(lower)
    upper_reference = first_related(upper_or_equal)
    if lower_reference < 0 and upper_reference < 0:
        report["updated_reference_vector_indices"].append(target_vector_index)
        report.update(
            {
                "success": True,
                "target_added_to_reference_indices": True,
                "diagnostic": (
                    "target had no related reference and was promoted"
                ),
            }
        )
        return report
    if lower_reference < 0 or upper_reference < 0:
        report.update(
            {
                "success": True,
                "selected_reference_vector_index": (
                    lower_reference
                    if lower_reference >= 0
                    else upper_reference
                ),
                "diagnostic": "single-sided reference selected",
            }
        )
        return report

    lower_intrudes = intrudes(target_vector_index, lower_reference)
    upper_intrudes = intrudes(target_vector_index, upper_reference)
    if not lower_intrudes and not upper_intrudes and not fallback:
        report.update(
            {
                "success": True,
                "deferred_for_fallback": True,
                "diagnostic": (
                    "two-sided non-intruding target deferred for fallback"
                ),
            }
        )
        return report

    lower_distance = (
        0.0
        if lower_intrudes
        else compute_candidate_reference_distance_m(
            candidates[lower_reference], candidates[target_vector_index]
        )
    )
    upper_distance = (
        0.0
        if upper_intrudes
        else compute_candidate_reference_distance_m(
            candidates[upper_reference], candidates[target_vector_index]
        )
    )
    lower_near = lower_distance < near_distance_m
    upper_near = upper_distance < near_distance_m
    if lower_near != upper_near:
        report.update(
            {
                "success": True,
                "selected_reference_vector_index": (
                    lower_reference if lower_near else upper_reference
                ),
                "diagnostic": "only one reference met the near threshold",
            }
        )
        return report
    if not lower_near:
        report.update(
            {
                "success": True,
                "selected_reference_vector_index": (
                    upper_reference
                    if upper_distance <= lower_distance
                    else lower_reference
                ),
                "diagnostic": (
                    "two distant references resolved by shortest distance"
                ),
            }
        )
        return report

    best_reference = -1
    best_boundary_index = 0
    best_squared_distance = 10000.0
    constraints = []
    for reference in (lower_reference, upper_reference):
        nearest_index, _ = min(
            enumerate(boundaries[reference]),
            key=lambda item: (
                (item[1][0] - grid_map_position_xy[0]) ** 2
                + (item[1][1] - grid_map_position_xy[1]) ** 2
            ),
        )
        nearest_squared_distance = (
            (boundaries[reference][nearest_index][0] - grid_map_position_xy[0])
            ** 2
            + (boundaries[reference][nearest_index][1] - grid_map_position_xy[1])
            ** 2
        )
        if nearest_squared_distance < best_squared_distance:
            best_squared_distance = nearest_squared_distance
            best_reference = reference
            best_boundary_index = nearest_index
        nearest = boundaries[best_reference][best_boundary_index]
        reference_height = heights[best_reference]
        constraints.extend(
            (
                (nearest[0], nearest[1], reference_height),
                (
                    grid_map_position_xy[0],
                    grid_map_position_xy[1],
                    reference_height,
                ),
            )
        )

    report.update(
        {
            "success": True,
            "selected_reference_vector_index": best_reference,
            "constraint_points_xyz": [list(point) for point in constraints],
            "diagnostic": (
                "two near references resolved by GridMap position"
            ),
        }
    )
    return report


def _compute_repair_y_norm(
    local_boundary: Sequence[tuple[float, float]],
    reference_length: float,
) -> float:
    eligible_y = [
        point[1]
        for point in local_boundary
        if 0.0 <= point[0] <= reference_length
        and 0.0 <= point[1] <= 5.0
    ]
    return min(5.0, min(eligible_y)) if eligible_y else 5.0


def _compute_repair_x_bounds(
    local_boundary: Sequence[tuple[float, float]],
    reference_length: float,
    y_norm: float,
) -> tuple[float, float]:
    if reference_length >= 10.0:
        return reference_length, 0.0
    y_filtered = [
        point for point in local_boundary if 0.0 <= point[1] <= y_norm
    ]
    if not y_filtered:
        half_length = reference_length * 0.5
        return half_length + 5.0, half_length - 5.0

    upper_limit = reference_length * 0.5 + 5.0
    upper_candidates = [
        point[0]
        for point in y_filtered
        if reference_length <= point[0] <= upper_limit
    ]
    x_max = min(upper_candidates) if upper_candidates else upper_limit

    lower_limit = reference_length * 0.5 - 5.0
    lower_candidates = [
        point[0]
        for point in y_filtered
        if lower_limit <= point[0] <= 0.0
    ]
    x_min = max(lower_candidates) if lower_candidates else lower_limit
    return x_max, x_min


def _cut_repair_hypothesis_by_contained_points(
    local_contained_points: Sequence[tuple[float, float]],
    y_norm: float,
    x_max: float,
    x_min: float,
    expand_to_08: bool,
) -> dict[str, Any]:
    factory_y_min = 0.0
    factory_y_max = _float32(y_norm)
    factory_x_max = _float32(x_max)
    factory_x_min = _float32(x_min)
    retained = [
        point
        for point in local_contained_points
        if (
            factory_y_min <= point[1] <= factory_y_max
            and factory_x_min <= point[0] <= factory_x_max
        )
    ]
    if not retained:
        return {
            "score": 0,
            "x_max": x_max,
            "x_min": x_min,
            "y_min": 0.0,
            "y_max": y_norm,
        }

    retained_x = [point[0] for point in retained]
    retained_y = [point[1] for point in retained]
    result = {
        "score": len(retained),
        "x_max": max(retained_x),
        "x_min": min(retained_x),
        "y_min": min(retained_y),
        "y_max": max(retained_y),
    }
    if expand_to_08:
        width = result["x_max"] - result["x_min"]
        height = result["y_max"] - result["y_min"]
        if (
            width < height
            and 0.03 <= width <= 0.4
            and 0.3 <= height < 0.8
        ):
            center = 0.5 * (result["y_min"] + result["y_max"])
            result["y_min"] = center - 0.4
            result["y_max"] = center + 0.4
        elif (
            width >= height
            and 0.03 <= height <= 0.4
            and 0.3 <= width < 0.8
        ):
            center = 0.5 * (result["x_min"] + result["x_max"])
            result["x_min"] = center - 0.4
            result["x_max"] = center + 0.4
    return result


def _repair_hypothesis_points(
    hypothesis: Mapping[str, Any],
    height: float,
    minimum_forward_extent_m: float,
) -> tuple[Point, Point, Point, Point]:
    origin = hypothesis["origin"]
    x_axis = hypothesis["x_axis"]
    y_axis = hypothesis["y_axis"]
    y_low = min(
        hypothesis["y_min"],
        hypothesis["y_max"] - minimum_forward_extent_m,
    )

    def world(x_value: float, y_value: float) -> Point:
        return (
            origin[0] + x_axis[0] * x_value + y_axis[0] * y_value,
            origin[1] + x_axis[1] * x_value + y_axis[1] * y_value,
            height,
        )

    return (
        world(hypothesis["x_max"], y_low),
        world(hypothesis["x_min"], y_low),
        world(hypothesis["x_min"], hypothesis["y_max"]),
        world(hypothesis["x_max"], hypothesis["y_max"]),
    )


def compute_and_update_candidate_quadrangles_from_reference(
    candidates: Sequence[CandidateTop],
    active_reference_vector_indices: Sequence[int],
    destination_vector_index: int,
    store_vector_index: int,
    reference_vector_index: int,
    grid_resolution_m: float,
    emit_all: bool,
    expand_to_08: bool = False,
    minimum_output_extent_m: float = FACTORY_REPAIR_MINIMUM_OUTPUT_EXTENT_M,
    minimum_forward_extent_m: float = (
        FACTORY_REPAIR_MINIMUM_FORWARD_EXTENT_M
    ),
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    working = list(candidates)
    report: dict[str, Any] = {
        "success": False,
        "emitted_quadrangle_count": 0,
        "hypothesis_scores": [0, 0, 0, 0],
        "registered_vector_indices": [],
        "error_vector_indices": [],
        "failed_reference_vector_indices": [],
        "diagnostic": "reference-based quadrangle update has not run",
    }
    if (
        destination_vector_index < 0
        or destination_vector_index >= len(working)
        or reference_vector_index < 0
        or reference_vector_index >= len(working)
    ):
        report["diagnostic"] = "quadrangle update index is outside the vector"
        return working, report
    if store_vector_index not in (destination_vector_index, len(working)):
        report["diagnostic"] = (
            "append store index must equal the current quadrangle count"
        )
        return working, report
    thresholds = (
        grid_resolution_m,
        minimum_output_extent_m,
        minimum_forward_extent_m,
        minimum_edge_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in thresholds):
        report["diagnostic"] = (
            "quadrangle update thresholds must be finite and positive"
        )
        return working, report
    if any(
        index < 0 or index >= len(working)
        for index in active_reference_vector_indices
    ):
        report["diagnostic"] = "active reference index is outside the vector"
        return working, report
    if any(
        not candidate.success or candidate.rectangle is None
        for candidate in working
    ):
        report["diagnostic"] = (
            "quadrangle update requires valid candidate rectangles"
        )
        return working, report

    destination = working[destination_vector_index]
    if not destination.contained_points:
        report["diagnostic"] = "destination contained-point cloud is empty"
        if not emit_all and store_vector_index == destination_vector_index:
            report["error_vector_indices"].append(destination_vector_index)
        elif emit_all:
            report["failed_reference_vector_indices"].append(
                reference_vector_index
            )
        return working, report
    if any(
        not all(math.isfinite(value) for value in point)
        for point in destination.contained_points
    ):
        report["diagnostic"] = "destination contained-point cloud is non-finite"
        return working, report

    world_boundary: list[tuple[float, float]] = []
    for index in active_reference_vector_indices:
        if index == reference_vector_index:
            continue
        world_boundary.extend(
            _sample_intrusion_boundary(working[index], grid_resolution_m)
        )

    assert working[reference_vector_index].rectangle is not None
    reference = working[reference_vector_index].rectangle.points
    edge_12 = _distance_xy(reference[1], reference[2])
    edge_01 = _distance_xy(reference[0], reference[1])
    ordered_reference = (
        (reference[0], reference[3], reference[2], reference[1])
        if edge_12 > edge_01
        else (reference[1], reference[0], reference[3], reference[2])
    )

    hypotheses: list[dict[str, Any]] = []
    for index in range(4):
        first = ordered_reference[index]
        second = ordered_reference[(index + 1) % 4]
        raw_x_axis = (first[0] - second[0], first[1] - second[1])
        reference_length = math.hypot(*raw_x_axis)
        x_axis = _normalize_xy(raw_x_axis, minimum_edge_m)
        if x_axis is None:
            report["diagnostic"] = (
                "reference quadrangle contains a degenerate edge"
            )
            return working, report
        y_axis = (-x_axis[1], x_axis[0])
        origin = (second[0], second[1])
        local_boundary = []
        for point in world_boundary:
            relative = (point[0] - origin[0], point[1] - origin[1])
            local_boundary.append(
                (
                    x_axis[0] * relative[0] + x_axis[1] * relative[1],
                    y_axis[0] * relative[0] + y_axis[1] * relative[1],
                )
            )
        y_norm = _compute_repair_y_norm(
            local_boundary, reference_length
        )
        x_max, x_min = _compute_repair_x_bounds(
            local_boundary, reference_length, y_norm
        )
        local_contained_points = []
        for point in destination.contained_points:
            relative = (point[0] - origin[0], point[1] - origin[1])
            local_contained_points.append(
                (
                    x_axis[0] * relative[0] + x_axis[1] * relative[1],
                    y_axis[0] * relative[0] + y_axis[1] * relative[1],
                )
            )
        hypothesis = _cut_repair_hypothesis_by_contained_points(
            local_contained_points,
            y_norm,
            x_max,
            x_min,
            expand_to_08,
        )
        hypothesis.update(
            {
                "origin": origin,
                "x_axis": x_axis,
                "y_axis": y_axis,
            }
        )
        hypotheses.append(hypothesis)
        report["hypothesis_scores"][index] = hypothesis["score"]

    assert destination.rectangle is not None
    destination_height = destination.rectangle.center[2]

    def commit(hypothesis_index: int, commit_index: int) -> None:
        repaired = _candidate_with_geometry(
            destination,
            _repair_hypothesis_points(
                hypotheses[hypothesis_index],
                destination_height,
                minimum_forward_extent_m,
            ),
            destination.contained_points,
            "factory reference-based quadrangle repair",
            hull=destination.hull,
            contained_points_are_recorded=(
                destination.contained_points_are_recorded
            ),
        )
        repaired = replace(repaired, candidate_index=commit_index)
        if commit_index == destination_vector_index:
            working[commit_index] = repaired
        else:
            working.append(repaired)
        report["registered_vector_indices"].append(commit_index)
        report["emitted_quadrangle_count"] += 1

    if not emit_all:
        best_index = max(
            range(4),
            key=lambda index: (hypotheses[index]["score"], -index),
        )
        if hypotheses[best_index]["score"] == 0:
            if destination_vector_index == store_vector_index:
                report["error_vector_indices"].append(
                    destination_vector_index
                )
            report["diagnostic"] = (
                "all reference-based quadrangle hypotheses were empty"
            )
            return working, report
        commit(best_index, store_vector_index)
        report["success"] = True
        report["diagnostic"] = (
            "earliest maximum-scoring quadrangle hypothesis committed"
        )
        return working, report

    for index, hypothesis in enumerate(hypotheses):
        if (
            hypothesis["score"] == 0
            or abs(hypothesis["x_max"] - hypothesis["x_min"])
            < minimum_output_extent_m
            or abs(hypothesis["y_max"] - hypothesis["y_min"])
            < minimum_output_extent_m
        ):
            continue
        commit_index = (
            store_vector_index
            if report["emitted_quadrangle_count"] == 0
            and store_vector_index == destination_vector_index
            else len(working)
        )
        commit(index, commit_index)
    if report["emitted_quadrangle_count"] == 0:
        report["failed_reference_vector_indices"].append(
            reference_vector_index
        )
        report["diagnostic"] = (
            "no emit-all quadrangle hypothesis met the 8 cm extent gate"
        )
        return working, report

    report["success"] = True
    report["diagnostic"] = (
        "all valid reference-based quadrangle hypotheses committed"
    )
    return working, report


def _delete_candidates_and_remap_indices(
    candidates: Sequence[CandidateTop],
    deletion_indices: Sequence[int],
    tracked_indices: Mapping[str, Sequence[int]],
) -> tuple[list[CandidateTop], dict[str, list[int]], dict[str, Any]]:
    deletion_set = {
        index for index in deletion_indices if 0 <= index < len(candidates)
    }
    old_to_new: dict[int, int] = {}
    compacted: list[CandidateTop] = []
    for old_index, candidate in enumerate(candidates):
        if old_index in deletion_set:
            continue
        old_to_new[old_index] = len(compacted)
        compacted.append(
            replace(candidate, candidate_index=len(compacted))
        )
    remapped = {
        name: [
            old_to_new[index]
            for index in indices
            if index in old_to_new
        ]
        for name, indices in tracked_indices.items()
    }
    return compacted, remapped, {
        "deleted_count": len(deletion_set),
        "deleted_vector_indices": sorted(deletion_set),
    }


def run_factory_final_suffix(
    candidates: Sequence[CandidateTop],
    grid_resolution_m: float,
    grid_map_position_xy: tuple[float, float],
    minimum_edge_m: float = 1.0e-6,
) -> tuple[list[CandidateTop], dict[str, Any]]:
    source_indices = [
        candidate.candidate_index
        for candidate in candidates
        if candidate.success and candidate.rectangle is not None
    ]
    working = [
        replace(candidate, candidate_index=index)
        for index, candidate in enumerate(
            candidate
            for candidate in candidates
            if candidate.success and candidate.rectangle is not None
        )
    ]
    report: dict[str, Any] = {
        "success": False,
        "input_candidate_count": len(candidates),
        "valid_input_candidate_count": len(working),
        "excluded_invalid_candidate_count": len(candidates) - len(working),
        "input_source_candidate_indices": source_indices,
        "narrow_normal_attempt_count": 0,
        "narrow_normal_success_count": 0,
        "fallback_target_count": 0,
        "fallback_success_count": 0,
        "wide_target_count": 0,
        "wide_reference_attempt_count": 0,
        "wide_successful_target_count": 0,
        "wide_attempts": [],
        "staged_promotion_count": 0,
        "staged_tail_count": 0,
        "deleted_error_quadrangle_count": 0,
        "deleted_degenerate_quadrangle_count": 0,
        "pitch_gated_is_error_quad": {
            "enabled": False,
            "reason": (
                "factory outer pitch predicate is not present in replay input"
            ),
        },
        "classification": {},
        "active_reference_vector_indices": [],
        "deferred_vector_indices": [],
        "successful_reference_vector_indices": [],
        "repaired_target_vector_indices": [],
        "wide_considered_reference_vector_indices": [],
        "wide_failed_reference_vector_indices": [],
        "final_candidate_count": 0,
        "final_candidate_top_rectangles": [],
        "diagnostic": "factory final suffix has not run",
    }
    if (
        not math.isfinite(grid_resolution_m)
        or grid_resolution_m <= 0.0
        or not all(math.isfinite(value) for value in grid_map_position_xy)
        or not math.isfinite(minimum_edge_m)
        or minimum_edge_m <= 0.0
    ):
        report["diagnostic"] = (
            "final suffix resolution, position, and edge threshold are invalid"
        )
        return [], report
    if not working:
        report["success"] = True
        report["diagnostic"] = (
            "factory final suffix completed with no valid quadrangles"
        )
        return [], report

    classification = classify_candidate_intrusions(
        working,
        grid_resolution_m,
        minimum_edge_m=minimum_edge_m,
    )
    report["classification"] = classification
    if not classification["success"]:
        report["diagnostic"] = classification["diagnostic"]
        return [], report

    active = list(classification["non_intruding_vector_indices"])
    deferred: list[int] = []
    successful_references: list[int] = []
    repaired_targets: list[int] = []
    wide_considered_references: list[int] = []
    wide_failed_references: list[int] = []
    staged_promoted: list[int] = []
    error_indices: list[int] = []

    def append_unique(indices: list[int], value: int) -> None:
        if value not in indices:
            indices.append(value)

    def process_narrow_target(target_index: int, fallback: bool) -> bool:
        nonlocal working, active
        selection = select_candidate_reference(
            working,
            active,
            target_index,
            grid_resolution_m,
            grid_map_position_xy,
            fallback,
            minimum_edge_m=minimum_edge_m,
        )
        if not selection["success"]:
            report["diagnostic"] = selection["diagnostic"]
            return False
        active = list(selection["updated_reference_vector_indices"])
        if selection["deferred_for_fallback"]:
            if fallback:
                report["diagnostic"] = (
                    "factory fallback unexpectedly deferred a narrow target"
                )
                return False
            deferred.append(target_index)
            return True
        reference_index = selection["selected_reference_vector_index"]
        if reference_index < 0:
            return True

        if not fallback:
            report["narrow_normal_attempt_count"] += 1
        working, update = (
            compute_and_update_candidate_quadrangles_from_reference(
                working,
                active,
                target_index,
                target_index,
                reference_index,
                grid_resolution_m,
                False,
                minimum_edge_m=minimum_edge_m,
            )
        )
        if not update["success"]:
            if update["error_vector_indices"]:
                error_indices.extend(update["error_vector_indices"])
                return True
            report["diagnostic"] = update["diagnostic"]
            return False
        for registered_index in update["registered_vector_indices"]:
            append_unique(active, registered_index)
        successful_references.append(reference_index)
        repaired_targets.append(target_index)
        if fallback:
            report["fallback_success_count"] += 1
        else:
            report["narrow_normal_success_count"] += 1
        return True

    for target_index in classification[
        "intruded_short_edge_vector_indices"
    ]:
        if not process_narrow_target(target_index, False):
            return [], report

    fallback_targets = list(deferred)
    report["fallback_target_count"] = len(fallback_targets)
    for target_index in fallback_targets:
        if not process_narrow_target(target_index, True):
            return [], report

    wide_targets = list(
        classification["intruded_long_edge_vector_indices"]
    )
    report["wide_target_count"] = len(wide_targets)
    tolerance_count = int(
        round(
            (
                FACTORY_WIDE_MAXIMUM_ATTEMPTED_HEIGHT_TOLERANCE_M
                - FACTORY_WIDE_INITIAL_HEIGHT_TOLERANCE_M
            )
            / FACTORY_WIDE_HEIGHT_TOLERANCE_STEP_M
        )
    ) + 1
    tolerances = [
        FACTORY_WIDE_INITIAL_HEIGHT_TOLERANCE_M
        + index * FACTORY_WIDE_HEIGHT_TOLERANCE_STEP_M
        for index in range(tolerance_count)
    ]
    for target_index in wide_targets:
        target_succeeded = False
        staged_indices: list[int] = []
        assert working[target_index].rectangle is not None
        target_height = working[target_index].rectangle.center[2]
        for tolerance in tolerances:
            eligible_references = [
                reference_index
                for reference_index in active
                if reference_index != target_index
                and _factory_wide_height_eligible(
                    working[reference_index].rectangle.center[2],
                    target_height,
                    tolerance,
                )
            ]
            for reference_index in eligible_references:
                report["wide_reference_attempt_count"] += 1
                wide_considered_references.append(reference_index)
                store_index = len(working) if target_succeeded else target_index
                working, update = (
                    compute_and_update_candidate_quadrangles_from_reference(
                        working,
                        active,
                        target_index,
                        store_index,
                        reference_index,
                        grid_resolution_m,
                        True,
                        minimum_edge_m=minimum_edge_m,
                    )
                )
                report["wide_attempts"].append(
                    {
                        "target_vector_index": target_index,
                        "reference_vector_index": reference_index,
                        "height_tolerance_m": tolerance,
                        "success": update["success"],
                        "hypothesis_scores": update["hypothesis_scores"],
                        "emitted_quadrangle_count": update[
                            "emitted_quadrangle_count"
                        ],
                        "registered_vector_indices": update[
                            "registered_vector_indices"
                        ],
                        "failed_reference_vector_indices": update[
                            "failed_reference_vector_indices"
                        ],
                    }
                )
                if not update["success"]:
                    if update["failed_reference_vector_indices"]:
                        wide_failed_references.extend(
                            update["failed_reference_vector_indices"]
                        )
                        continue
                    report["diagnostic"] = update["diagnostic"]
                    return [], report
                target_succeeded = True
                successful_references.append(reference_index)
                staged_indices.extend(
                    update["registered_vector_indices"]
                )
            if target_succeeded:
                break
        if not target_succeeded:
            error_indices.append(target_index)
            continue
        report["wide_successful_target_count"] += 1
        for staged_index in staged_indices:
            append_unique(active, staged_index)
            staged_promoted.append(staged_index)
        report["staged_promotion_count"] += len(staged_indices)

    tracked = {
        "active": active,
        "deferred": deferred,
        "successful_references": successful_references,
        "repaired_targets": repaired_targets,
        "wide_considered_references": wide_considered_references,
        "wide_failed_references": wide_failed_references,
        "staged_promoted": staged_promoted,
    }
    working, tracked, deletion = _delete_candidates_and_remap_indices(
        working, error_indices, tracked
    )
    report["deleted_error_quadrangle_count"] = deletion["deleted_count"]

    degenerate_indices = []
    for active_index in tracked["active"]:
        assert working[active_index].rectangle is not None
        points = working[active_index].rectangle.points
        first_edge_squared = (
            (points[0][0] - points[1][0]) ** 2
            + (points[0][1] - points[1][1]) ** 2
        )
        if first_edge_squared < FACTORY_DEGENERATE_FIRST_EDGE_SQUARED_M2:
            append_unique(degenerate_indices, active_index)
    working, tracked, deletion = _delete_candidates_and_remap_indices(
        working, degenerate_indices, tracked
    )
    report["deleted_degenerate_quadrangle_count"] = (
        deletion["deleted_count"]
    )

    final_candidates = [working[index] for index in tracked["active"]]
    report.update(
        {
            "success": True,
            "active_reference_vector_indices": tracked["active"],
            "deferred_vector_indices": tracked["deferred"],
            "successful_reference_vector_indices": tracked[
                "successful_references"
            ],
            "repaired_target_vector_indices": tracked["repaired_targets"],
            "wide_considered_reference_vector_indices": tracked[
                "wide_considered_references"
            ],
            "wide_failed_reference_vector_indices": tracked[
                "wide_failed_references"
            ],
            "staged_promoted_vector_indices": tracked["staged_promoted"],
            "staged_tail_count": len(tracked["staged_promoted"]),
            "final_candidate_count": len(final_candidates),
            "final_candidate_top_rectangles": [
                {
                    "candidate_index": candidate.candidate_index,
                    "center_xyz": list(candidate.rectangle.center),
                    "area_3d_m2": candidate.rectangle.area_3d_m2,
                    "points_xyz": [
                        list(point)
                        for point in candidate.rectangle.points
                    ],
                }
                for candidate in final_candidates
            ],
            "diagnostic": (
                "factory final suffix completed with pitch-gated "
                "isErrorQuad disabled"
            ),
        }
    )
    return final_candidates, report


def _best_vertex_alignment(
    candidate: Sequence[Point], factory: Sequence[Point]
) -> dict[str, Any]:
    best: Optional[tuple[float, float, bool, int, tuple[Point, ...]]] = None
    for reversed_order in (False, True):
        for shift in range(4):
            aligned = tuple(
                factory[(shift - index) % 4] if reversed_order
                else factory[(index + shift) % 4]
                for index in range(4)
            )
            distances = tuple(
                _distance(left, right) for left, right in zip(candidate, aligned)
            )
            rms = math.sqrt(math.fsum(value * value for value in distances) / 4.0)
            maximum = max(distances)
            key = (rms, maximum, reversed_order, shift, aligned)
            if best is None or key[:4] < best[:4]:
                best = key
    assert best is not None
    return {
        "rms_m": best[0],
        "maximum_m": best[1],
        "factory_reversed": best[2],
        "factory_cyclic_shift": best[3],
    }


def _relative_error(left: float, right: float) -> float:
    scale = max(abs(left), abs(right), 1.0e-12)
    return abs(left - right) / scale


def _pair_metrics(
    candidate: CandidateTop,
    factory_group_index: int,
    factory: Rectangle,
    thresholds: Thresholds,
) -> dict[str, Any]:
    assert candidate.rectangle is not None
    top = candidate.rectangle
    vertex = _best_vertex_alignment(top.points, factory.points)
    center_xy = _distance_xy(top.center, factory.center)
    center_3d = _distance(top.center, factory.center)
    height = abs(top.center[2] - factory.center[2])
    area_absolute = abs(top.area_3d_m2 - factory.area_3d_m2)
    area_relative = _relative_error(top.area_3d_m2, factory.area_3d_m2)
    score = (
        center_xy / thresholds.center_xy_m
        + height / thresholds.height_m
        + area_relative / thresholds.area_relative
        + vertex["rms_m"] / thresholds.vertex_rms_m
    )
    accepted = (
        center_xy <= thresholds.center_xy_m
        and height <= thresholds.height_m
        and area_relative <= thresholds.area_relative
        and vertex["rms_m"] <= thresholds.vertex_rms_m
    )
    return {
        "candidate_index": candidate.candidate_index,
        "factory_group_index": factory_group_index,
        "score": score,
        "accepted": accepted,
        "center_xy_error_m": center_xy,
        "center_3d_error_m": center_3d,
        "height_error_m": height,
        "area_absolute_error_m2": area_absolute,
        "area_relative_error": area_relative,
        "vertex_distance": vertex,
    }


def _greedy_assignment(
    candidates: Sequence[CandidateTop],
    factories: Sequence[tuple[int, Rectangle]],
    thresholds: Thresholds,
) -> tuple[list[dict[str, Any]], list[int], list[int]]:
    edges = [
        _pair_metrics(candidate, group_index, factory, thresholds)
        for candidate in candidates
        if candidate.success
        for group_index, factory in factories
    ]
    edges.sort(
        key=lambda row: (
            row["score"],
            row["candidate_index"],
            row["factory_group_index"],
        )
    )
    used_candidates: set[int] = set()
    used_factories: set[int] = set()
    matches = []
    for edge in edges:
        candidate_index = edge["candidate_index"]
        factory_index = edge["factory_group_index"]
        if candidate_index in used_candidates or factory_index in used_factories:
            continue
        used_candidates.add(candidate_index)
        used_factories.add(factory_index)
        matches.append(edge)
    matches.sort(key=lambda row: (row["factory_group_index"], row["candidate_index"]))
    valid_indices = {candidate.candidate_index for candidate in candidates if candidate.success}
    factory_indices = {index for index, _ in factories}
    return (
        matches,
        sorted(valid_indices - used_candidates),
        sorted(factory_indices - used_factories),
    )


def _minimum_xy_gap(left: Rectangle, right: Rectangle) -> float:
    return min(_distance_xy(a, b) for a in left.points for b in right.points)


def _merge_clues(
    candidates: Sequence[CandidateTop],
    factories: Sequence[tuple[int, Rectangle]],
    thresholds: Thresholds,
) -> list[dict[str, Any]]:
    clues = []
    valid = [candidate for candidate in candidates if candidate.success]
    for left, right in itertools.combinations(valid, 2):
        assert left.rectangle is not None and right.rectangle is not None
        left_top = left.rectangle
        right_top = right.rectangle
        gap = _minimum_xy_gap(left_top, right_top)
        candidate_height_delta = abs(left_top.center[2] - right_top.center[2])
        combined_area = left_top.area_3d_m2 + right_top.area_3d_m2
        if combined_area <= 0.0:
            continue
        combined_center = tuple(
            (
                left_top.center[axis] * left_top.area_3d_m2
                + right_top.center[axis] * right_top.area_3d_m2
            )
            / combined_area
            for axis in range(3)
        )
        for factory_index, factory in factories:
            height_error = abs(combined_center[2] - factory.center[2])
            center_error = _distance_xy(combined_center, factory.center)
            combined_area_error = _relative_error(combined_area, factory.area_3d_m2)
            best_single_area_error = min(
                _relative_error(left_top.area_3d_m2, factory.area_3d_m2),
                _relative_error(right_top.area_3d_m2, factory.area_3d_m2),
            )
            improves_area = combined_area_error < best_single_area_error
            if not (
                candidate_height_delta <= thresholds.merge_height_m
                and height_error <= thresholds.merge_height_m
                and center_error <= thresholds.merge_center_xy_m
                and combined_area_error <= thresholds.merge_area_relative
                and gap <= thresholds.merge_gap_xy_m
                and improves_area
            ):
                continue
            score = (
                height_error / thresholds.merge_height_m
                + center_error / thresholds.merge_center_xy_m
                + combined_area_error / thresholds.merge_area_relative
                + gap / thresholds.merge_gap_xy_m
            )
            clues.append(
                {
                    "candidate_indices": [
                        left.candidate_index,
                        right.candidate_index,
                    ],
                    "factory_group_index": factory_index,
                    "score": score,
                    "candidate_height_delta_m": candidate_height_delta,
                    "combined_center_xy_error_m": center_error,
                    "combined_height_error_m": height_error,
                    "combined_area_m2": combined_area,
                    "factory_area_m2": factory.area_3d_m2,
                    "combined_area_relative_error": combined_area_error,
                    "best_single_area_relative_error": best_single_area_error,
                    "minimum_candidate_gap_xy_m": gap,
                    "interpretation": (
                        "diagnostic pair only; may indicate factory merge/cut post-processing"
                    ),
                }
            )
    clues.sort(
        key=lambda row: (
            row["score"],
            row["factory_group_index"],
            row["candidate_indices"],
        )
    )
    return clues[: thresholds.max_merge_clues_per_frame]


def _looks_like_factory_sentinel(
    points: Sequence[Point],
    absolute_tolerance: float,
    relative_tolerance: float,
) -> bool:
    analysis = parity._factory_analysis(  # Reuse the established sentinel contract.
        points,
        absolute_tolerance,
        relative_tolerance,
        absolute_tolerance * absolute_tolerance,
    )
    return bool(analysis["sentinel"]["all_checks_pass"])


def _factory_staged_tail_count(
    raw_groups: Sequence[Sequence[Point]],
    sentinel_present: bool,
    absolute_tolerance: float,
) -> Optional[int]:
    if not sentinel_present or not raw_groups or not raw_groups[0]:
        return None
    encoded = raw_groups[0][0][0] - 0.001
    nearest = int(round(encoded))
    if nearest < 0 or abs(encoded - nearest) > absolute_tolerance:
        return None
    return nearest


def _factory_wide_height_eligible(
    reference_height: float,
    target_height: float,
    tolerance: float,
) -> bool:
    return abs(reference_height - target_height) < tolerance


def _frame_diagnostic(
    replay_frame: parity.ReplayFrame,
    output_frame: parity.ReplayOutputFrame,
    absolute_tolerance: float,
    relative_tolerance: float,
    thresholds: Thresholds,
) -> dict[str, Any]:
    raw_groups = [
        tuple(replay_frame.factory_points[index:index + 4])
        for index in range(0, len(replay_frame.factory_points), 4)
    ]
    sentinel_present = _looks_like_factory_sentinel(
        replay_frame.factory_points, absolute_tolerance, relative_tolerance
    )
    factory_staged_tail_count = _factory_staged_tail_count(
        raw_groups, sentinel_present, absolute_tolerance
    )
    first_geometry_index = 1 if sentinel_present else 0
    factories = [
        (index, _rectangle(group))
        for index, group in enumerate(raw_groups)
        if index >= first_geometry_index
    ]
    tops = [
        candidate_top_rectangle(candidate, index, thresholds.minimum_edge_m)
        for index, candidate in enumerate(output_frame.candidates)
    ]
    matches, unmatched_candidates, unmatched_factories = _greedy_assignment(
        tops, factories, thresholds
    )
    corrected_tops, pose_correction = correct_candidate_top_rectangles(
        tops, thresholds.minimum_edge_m
    )
    corrected_matches, corrected_unmatched_candidates, corrected_unmatched_factories = (
        _greedy_assignment(corrected_tops, factories, thresholds)
    )
    pose_correction["matching"] = {
        "method": "deterministic_global_edge_greedy",
        "matched_pair_count": len(corrected_matches),
        "accepted_pair_count": sum(
            match["accepted"] for match in corrected_matches
        ),
        "rejected_pair_count": len(corrected_matches)
        - sum(match["accepted"] for match in corrected_matches),
        "unmatched_valid_candidate_indices": corrected_unmatched_candidates,
        "unmatched_factory_group_indices": corrected_unmatched_factories,
        "pairs": corrected_matches,
    }
    pose_correction["corrected_candidate_top_rectangles"] = [
        {
            "candidate_index": top.candidate_index,
            "success": top.success,
            "diagnostic": top.diagnostic,
            "center_xyz": (
                list(top.rectangle.center) if top.rectangle is not None else None
            ),
            "area_3d_m2": (
                top.rectangle.area_3d_m2 if top.rectangle is not None else None
            ),
            "points_xyz": (
                [list(point) for point in top.rectangle.points]
                if top.rectangle is not None else []
            ),
        }
        for top in corrected_tops
    ]
    side_direction = (
        pose_correction["corrected_stair_direction_xy"]
        or pose_correction["initial_stair_direction_xy"]
        or (0.0, 0.0)
    )
    pre_merge_cut_tops, pre_merge_cut = cut_candidate_tops_by_x(
        corrected_tops,
        side_direction,
        minimum_edge_m=thresholds.minimum_edge_m,
    )
    pre_merge_cut_matches, cut_unmatched_candidates, cut_unmatched_factories = (
        _greedy_assignment(pre_merge_cut_tops, factories, thresholds)
    )
    pre_merge_cut["matching"] = {
        "method": "deterministic_global_edge_greedy",
        "matched_pair_count": len(pre_merge_cut_matches),
        "accepted_pair_count": sum(
            match["accepted"] for match in pre_merge_cut_matches
        ),
        "rejected_pair_count": len(pre_merge_cut_matches)
        - sum(match["accepted"] for match in pre_merge_cut_matches),
        "unmatched_valid_candidate_indices": cut_unmatched_candidates,
        "unmatched_factory_group_indices": cut_unmatched_factories,
        "pairs": pre_merge_cut_matches,
    }
    merged_tops, same_stair_merge = merge_candidate_tops_at_same_stair(
        pre_merge_cut_tops,
        side_direction,
        thresholds.minimum_edge_m,
    )
    merge_matches, merge_unmatched_candidates, merge_unmatched_factories = (
        _greedy_assignment(merged_tops, factories, thresholds)
    )
    same_stair_merge["matching"] = {
        "method": "deterministic_global_edge_greedy",
        "matched_pair_count": len(merge_matches),
        "accepted_pair_count": sum(match["accepted"] for match in merge_matches),
        "rejected_pair_count": len(merge_matches)
        - sum(match["accepted"] for match in merge_matches),
        "unmatched_valid_candidate_indices": merge_unmatched_candidates,
        "unmatched_factory_group_indices": merge_unmatched_factories,
        "pairs": merge_matches,
    }
    side_filtered_tops, side_filter = ignore_unnecessary_candidate_tops(
        merged_tops,
        side_direction,
        FACTORY_SIDE_DISTANCE_THRESHOLD_M,
        thresholds.minimum_edge_m,
    )
    side_matches, side_unmatched_candidates, side_unmatched_factories = (
        _greedy_assignment(side_filtered_tops, factories, thresholds)
    )
    side_filter["matching"] = {
        "method": "deterministic_global_edge_greedy",
        "matched_pair_count": len(side_matches),
        "accepted_pair_count": sum(match["accepted"] for match in side_matches),
        "rejected_pair_count": len(side_matches)
        - sum(match["accepted"] for match in side_matches),
        "unmatched_valid_candidate_indices": side_unmatched_candidates,
        "unmatched_factory_group_indices": side_unmatched_factories,
        "pairs": side_matches,
    }
    if side_filter["selected_stair_direction_xy"] is not None:
        side_direction = tuple(side_filter["selected_stair_direction_xy"])
    intrusion_classification = classify_candidate_intrusions(
        side_filtered_tops,
        replay_frame.grid_resolution_m,
        minimum_edge_m=thresholds.minimum_edge_m,
    )
    final_tops, final_suffix = run_factory_final_suffix(
        side_filtered_tops,
        replay_frame.grid_resolution_m,
        (
            replay_frame.grid_center_xyz[0],
            replay_frame.grid_center_xyz[1],
        ),
        thresholds.minimum_edge_m,
    )
    final_matches, final_unmatched_candidates, final_unmatched_factories = (
        _greedy_assignment(final_tops, factories, thresholds)
    )
    final_suffix["matching"] = {
        "method": "deterministic_global_edge_greedy",
        "matched_pair_count": len(final_matches),
        "accepted_pair_count": sum(
            match["accepted"] for match in final_matches
        ),
        "rejected_pair_count": len(final_matches)
        - sum(match["accepted"] for match in final_matches),
        "unmatched_valid_candidate_indices": final_unmatched_candidates,
        "unmatched_factory_group_indices": final_unmatched_factories,
        "pairs": final_matches,
    }
    final_suffix["candidate_minus_factory_geometry"] = (
        len(final_tops) - len(factories)
    )
    final_suffix["factory_staged_tail_count"] = factory_staged_tail_count
    final_suffix["staged_tail_count_minus_factory"] = (
        final_suffix["staged_tail_count"] - factory_staged_tail_count
        if factory_staged_tail_count is not None
        else None
    )
    if (
        factory_staged_tail_count is not None
        and factory_staged_tail_count <= len(factories)
        and final_suffix["staged_tail_count"] <= len(final_tops)
    ):
        factory_base_count = len(factories) - factory_staged_tail_count
        candidate_base_count = (
            len(final_tops) - final_suffix["staged_tail_count"]
        )
        base_matches, base_unmatched_candidates, base_unmatched_factories = (
            _greedy_assignment(
                final_tops[:candidate_base_count],
                factories[:factory_base_count],
                thresholds,
            )
        )
        staged_matches, staged_unmatched_candidates, staged_unmatched_factories = (
            _greedy_assignment(
                final_tops[candidate_base_count:],
                factories[factory_base_count:],
                thresholds,
            )
        )
        final_suffix["base_matching"] = {
            "candidate_count": candidate_base_count,
            "factory_count": factory_base_count,
            "matched_pair_count": len(base_matches),
            "accepted_pair_count": sum(
                match["accepted"] for match in base_matches
            ),
            "unmatched_valid_candidate_indices": base_unmatched_candidates,
            "unmatched_factory_group_indices": base_unmatched_factories,
            "pairs": base_matches,
        }
        final_suffix["staged_tail_matching"] = {
            "candidate_count": final_suffix["staged_tail_count"],
            "factory_count": factory_staged_tail_count,
            "matched_pair_count": len(staged_matches),
            "accepted_pair_count": sum(
                match["accepted"] for match in staged_matches
            ),
            "unmatched_valid_candidate_indices": staged_unmatched_candidates,
            "unmatched_factory_group_indices": staged_unmatched_factories,
            "pairs": staged_matches,
        }
    else:
        final_suffix["base_matching"] = None
        final_suffix["staged_tail_matching"] = None
    invalid_indices = [top.candidate_index for top in tops if not top.success]
    accepted_count = sum(match["accepted"] for match in matches)
    frame_132 = (
        replay_frame.selected_index == 132
        or replay_frame.stamp_ns == KNOWN_FRAME_132_STAMP_NS
    )
    degenerates = [
        {
            "candidate_index": top.candidate_index,
            "diagnostic": top.diagnostic,
            "size": list(top.size),
            "translation": list(top.translation),
            "hull_point_count": top.hull_point_count,
            "hull_area_3d_m2": top.hull_area_3d_m2,
            "matches_known_empty_zero_size_signature": (
                top.hull_point_count == 0
                and top.size[0] <= thresholds.minimum_edge_m
                and top.size[1] <= thresholds.minimum_edge_m
            ),
        }
        for top in tops
        if not top.success
    ]
    return {
        "case": replay_frame.case,
        "stamp_ns": replay_frame.stamp_ns,
        "selected_index": replay_frame.selected_index,
        "retained": output_frame.retained,
        "counts": {
            "factory_groups_raw": len(raw_groups),
            "factory_geometry_groups": len(factories),
            "factory_sentinel_present": sentinel_present,
            "factory_staged_tail_count": factory_staged_tail_count,
            "core_candidates": len(tops),
            "valid_candidate_top_rectangles": len(tops) - len(invalid_indices),
            "candidate_minus_factory_raw": len(tops) - len(raw_groups),
            "candidate_minus_factory_geometry": len(tops) - len(factories),
        },
        "matching": {
            "method": "deterministic_global_edge_greedy",
            "matched_pair_count": len(matches),
            "accepted_pair_count": accepted_count,
            "rejected_pair_count": len(matches) - accepted_count,
            "unmatched_valid_candidate_indices": unmatched_candidates,
            "unmatched_factory_group_indices": unmatched_factories,
            "pairs": matches,
        },
        "candidate_top_rectangles": [
            {
                "candidate_index": top.candidate_index,
                "success": top.success,
                "diagnostic": top.diagnostic,
                "size": list(top.size),
                "translation": list(top.translation),
                "hull_point_count": top.hull_point_count,
                "hull_area_3d_m2": top.hull_area_3d_m2,
                "center_xyz": (
                    list(top.rectangle.center) if top.rectangle is not None else None
                ),
                "area_3d_m2": (
                    top.rectangle.area_3d_m2 if top.rectangle is not None else None
                ),
                "signed_xy_area_m2": (
                    top.rectangle.signed_xy_area_m2
                    if top.rectangle is not None else None
                ),
                "points_xyz": (
                    [list(point) for point in top.rectangle.points]
                    if top.rectangle is not None else []
                ),
            }
            for top in tops
        ],
        "pose_correction": pose_correction,
        "pre_merge_cut_by_x": pre_merge_cut,
        "same_stair_merge": same_stair_merge,
        "side_distance_filter": side_filter,
        "intrusion_classification": intrusion_classification,
        "factory_final_suffix": final_suffix,
        "merge_clues": _merge_clues(tops, factories, thresholds),
        "frame_132": {
            "is_target_frame": frame_132,
            "known_stamp_ns": KNOWN_FRAME_132_STAMP_NS,
            "degenerate_candidate_count": len(degenerates),
            "degenerate_candidates": degenerates,
        },
    }


def build_diagnostic_from_parsed(
    pack: parity.ReplayPack,
    output_frames: Sequence[parity.ReplayOutputFrame],
    thresholds: Thresholds = Thresholds(),
    replay_jsonl_sha256: Optional[str] = None,
    replay_jsonl_size_bytes: Optional[int] = None,
) -> dict[str, Any]:
    thresholds.validate()
    pack_by_key = {frame.key: frame for frame in pack.frames}
    output_by_key = {frame.key: frame for frame in output_frames}
    if set(pack_by_key) != set(output_by_key):
        missing = sorted(set(pack_by_key) - set(output_by_key))
        extra = sorted(set(output_by_key) - set(pack_by_key))
        raise DiagnosticError(
            f"replay/JSONL frame sets differ: missing={missing!r}, extra={extra!r}"
        )
    frames = []
    for replay_frame in pack.frames:
        output_frame = output_by_key[replay_frame.key]
        expected_factory_count = len(replay_frame.factory_points) // 4
        if output_frame.factory_group_count != expected_factory_count:
            raise DiagnosticError(
                f"{replay_frame.key!r}: factory_group_count mismatch"
            )
        if output_frame.retained != replay_frame.expected_retained_count:
            raise DiagnosticError(
                f"{replay_frame.key!r}: retained sample count mismatch"
            )
        frames.append(
            _frame_diagnostic(
                replay_frame,
                output_frame,
                pack.header.absolute_tolerance,
                pack.header.relative_tolerance,
                thresholds,
            )
        )

    target_frames = [frame for frame in frames if frame["frame_132"]["is_target_frame"]]
    return {
        "schema": SCHEMA_NAME,
        "scope": {
            "kind": "offline_candidate_to_factory_geometry_diagnostic",
            "requires_network": False,
            "requires_robot": False,
            "modifies_core_algorithm": False,
            "factory_postprocessing_recovered": False,
            "recovered_stages": [
                "computeStairPose",
                "correctQuadPose",
                "cutByX",
                "computeDistanceVector",
                "mergeQuadsAtSameStair",
                "ignoreUnnecessaryQuads",
                "pcl2vectors",
                "sortVectors",
                "configIntrusions",
                "getReferenceIndice",
                "computeAndUpdateNewQuad",
                "normal_reference_repair",
                "fallback_reference_repair",
                "wide_reference_repair",
                "dealErrorIndices",
                "degenerate_first_edge_cleanup",
            ],
            "unrecovered_stages": [
                "pitch_gated_isErrorQuad_outer_predicate",
            ],
        },
        "inputs": {
            "replay_pack": {
                "sha256": pack.sha256,
                "size_bytes": pack.size_bytes,
                "schema": "x30rpl/1.0",
            },
            "replay_jsonl": {
                "sha256": replay_jsonl_sha256,
                "size_bytes": replay_jsonl_size_bytes,
            },
        },
        "thresholds": thresholds.as_json(),
        "totals": {
            "frame_count": len(frames),
            "factory_groups_raw": sum(
                frame["counts"]["factory_groups_raw"] for frame in frames
            ),
            "factory_geometry_groups": sum(
                frame["counts"]["factory_geometry_groups"] for frame in frames
            ),
            "core_candidates": sum(
                frame["counts"]["core_candidates"] for frame in frames
            ),
            "valid_candidate_top_rectangles": sum(
                frame["counts"]["valid_candidate_top_rectangles"] for frame in frames
            ),
            "matched_pairs": sum(
                frame["matching"]["matched_pair_count"] for frame in frames
            ),
            "accepted_pairs": sum(
                frame["matching"]["accepted_pair_count"] for frame in frames
            ),
            "pose_correction_success_frames": sum(
                frame["pose_correction"]["success"] for frame in frames
            ),
            "pose_corrected_matched_pairs": sum(
                frame["pose_correction"]["matching"]["matched_pair_count"]
                for frame in frames
            ),
            "pose_corrected_accepted_pairs": sum(
                frame["pose_correction"]["matching"]["accepted_pair_count"]
                for frame in frames
            ),
            "pre_merge_cut_success_frames": sum(
                frame["pre_merge_cut_by_x"]["success"] for frame in frames
            ),
            "pre_merge_cut_changed_candidates": sum(
                frame["pre_merge_cut_by_x"]["changed_candidate_count"]
                for frame in frames
            ),
            "pre_merge_cut_accepted_pairs": sum(
                frame["pre_merge_cut_by_x"]["matching"]["accepted_pair_count"]
                for frame in frames
            ),
            "same_stair_merge_success_frames": sum(
                frame["same_stair_merge"]["success"] for frame in frames
            ),
            "same_stair_merged_pairs": sum(
                frame["same_stair_merge"]["merged_pair_count"]
                for frame in frames
            ),
            "same_stair_merge_output_candidates": sum(
                frame["same_stair_merge"]["output_candidate_count"]
                for frame in frames
            ),
            "same_stair_merge_accepted_pairs": sum(
                frame["same_stair_merge"]["matching"]["accepted_pair_count"]
                for frame in frames
            ),
            "side_distance_filter_success_frames": sum(
                frame["side_distance_filter"]["success"] for frame in frames
            ),
            "side_distance_filtered_candidates": sum(
                frame["side_distance_filter"]["matching"]["matched_pair_count"]
                + len(
                    frame["side_distance_filter"]["matching"][
                        "unmatched_valid_candidate_indices"
                    ]
                )
                for frame in frames
            ),
            "side_distance_filtered_matched_pairs": sum(
                frame["side_distance_filter"]["matching"]["matched_pair_count"]
                for frame in frames
            ),
            "side_distance_filtered_accepted_pairs": sum(
                frame["side_distance_filter"]["matching"]["accepted_pair_count"]
                for frame in frames
            ),
            "intrusion_classification_success_frames": sum(
                frame["intrusion_classification"]["success"] for frame in frames
            ),
            "intrusion_non_intruding_candidates": sum(
                len(
                    frame["intrusion_classification"][
                        "non_intruding_vector_indices"
                    ]
                )
                for frame in frames
            ),
            "intrusion_short_edge_candidates": sum(
                len(
                    frame["intrusion_classification"][
                        "intruded_short_edge_vector_indices"
                    ]
                )
                for frame in frames
            ),
            "intrusion_long_edge_candidates": sum(
                len(
                    frame["intrusion_classification"][
                        "intruded_long_edge_vector_indices"
                    ]
                )
                for frame in frames
            ),
            "factory_final_suffix_success_frames": sum(
                frame["factory_final_suffix"]["success"]
                for frame in frames
            ),
            "factory_final_suffix_candidates": sum(
                frame["factory_final_suffix"]["final_candidate_count"]
                for frame in frames
            ),
            "factory_final_suffix_matched_pairs": sum(
                frame["factory_final_suffix"]["matching"][
                    "matched_pair_count"
                ]
                for frame in frames
            ),
            "factory_final_suffix_accepted_pairs": sum(
                frame["factory_final_suffix"]["matching"][
                    "accepted_pair_count"
                ]
                for frame in frames
            ),
            "factory_final_suffix_exact_count_frames": sum(
                frame["factory_final_suffix"][
                    "candidate_minus_factory_geometry"
                ]
                == 0
                for frame in frames
            ),
            "factory_final_suffix_narrow_successes": sum(
                frame["factory_final_suffix"][
                    "narrow_normal_success_count"
                ]
                for frame in frames
            ),
            "factory_final_suffix_fallback_successes": sum(
                frame["factory_final_suffix"]["fallback_success_count"]
                for frame in frames
            ),
            "factory_final_suffix_wide_successful_targets": sum(
                frame["factory_final_suffix"][
                    "wide_successful_target_count"
                ]
                for frame in frames
            ),
            "factory_staged_tail_count": sum(
                frame["factory_final_suffix"]["factory_staged_tail_count"]
                for frame in frames
                if frame["factory_final_suffix"]["factory_staged_tail_count"]
                is not None
            ),
            "factory_final_suffix_staged_tail_count": sum(
                frame["factory_final_suffix"]["staged_tail_count"]
                for frame in frames
            ),
            "factory_final_suffix_exact_staged_tail_frames": sum(
                frame["factory_final_suffix"][
                    "staged_tail_count_minus_factory"
                ]
                == 0
                for frame in frames
            ),
            "factory_final_suffix_base_candidates": sum(
                frame["factory_final_suffix"]["base_matching"][
                    "candidate_count"
                ]
                for frame in frames
                if frame["factory_final_suffix"]["base_matching"] is not None
            ),
            "factory_base_geometry_groups": sum(
                frame["factory_final_suffix"]["base_matching"][
                    "factory_count"
                ]
                for frame in frames
                if frame["factory_final_suffix"]["base_matching"] is not None
            ),
            "factory_final_suffix_base_accepted_pairs": sum(
                frame["factory_final_suffix"]["base_matching"][
                    "accepted_pair_count"
                ]
                for frame in frames
                if frame["factory_final_suffix"]["base_matching"] is not None
            ),
            "factory_final_suffix_staged_accepted_pairs": sum(
                frame["factory_final_suffix"]["staged_tail_matching"][
                    "accepted_pair_count"
                ]
                for frame in frames
                if frame["factory_final_suffix"]["staged_tail_matching"]
                is not None
            ),
            "factory_final_suffix_deleted_errors": sum(
                frame["factory_final_suffix"][
                    "deleted_error_quadrangle_count"
                ]
                for frame in frames
            ),
            "merge_clue_count": sum(len(frame["merge_clues"]) for frame in frames),
            "degenerate_candidate_count": sum(
                frame["frame_132"]["degenerate_candidate_count"] for frame in frames
            ),
        },
        "frame_132_summary": {
            "target_frame_count": len(target_frames),
            "targets": [
                {
                    "case": frame["case"],
                    "stamp_ns": frame["stamp_ns"],
                    "selected_index": frame["selected_index"],
                    "degenerate_candidate_count": frame["frame_132"][
                        "degenerate_candidate_count"
                    ],
                    "degenerate_candidates": frame["frame_132"][
                        "degenerate_candidates"
                    ],
                }
                for frame in target_frames
            ],
        },
        "frames": frames,
    }


def build_diagnostic(
    replay_pack_bytes: bytes,
    replay_jsonl_bytes: bytes,
    thresholds: Thresholds = Thresholds(),
) -> dict[str, Any]:
    pack = parity.parse_replay_pack(replay_pack_bytes)
    output_frames = parity.parse_replay_jsonl(replay_jsonl_bytes)
    return build_diagnostic_from_parsed(
        pack,
        output_frames,
        thresholds,
        replay_jsonl_sha256=parity._sha256(replay_jsonl_bytes),
        replay_jsonl_size_bytes=len(replay_jsonl_bytes),
    )


def render_diagnostic(report: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(
            report,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("ascii")


def diagnose_files(
    replay_pack_path: Path,
    replay_jsonl_path: Path,
    output_path: Path,
    thresholds: Thresholds = Thresholds(),
) -> dict[str, Any]:
    resolved_output = output_path.resolve()
    if resolved_output in {replay_pack_path.resolve(), replay_jsonl_path.resolve()}:
        raise DiagnosticError("output path must not overwrite either input")
    report = build_diagnostic(
        replay_pack_path.read_bytes(),
        replay_jsonl_path.read_bytes(),
        thresholds,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(render_diagnostic(report))
    return report


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-pack", required=True, type=Path)
    parser.add_argument("--input-jsonl", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--center-xy-m", type=float, default=Thresholds.center_xy_m)
    parser.add_argument("--height-m", type=float, default=Thresholds.height_m)
    parser.add_argument(
        "--area-relative", type=float, default=Thresholds.area_relative
    )
    parser.add_argument(
        "--vertex-rms-m", type=float, default=Thresholds.vertex_rms_m
    )
    parser.add_argument(
        "--max-merge-clues-per-frame",
        type=int,
        default=Thresholds.max_merge_clues_per_frame,
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    thresholds = Thresholds(
        center_xy_m=args.center_xy_m,
        height_m=args.height_m,
        area_relative=args.area_relative,
        vertex_rms_m=args.vertex_rms_m,
        max_merge_clues_per_frame=args.max_merge_clues_per_frame,
    )
    try:
        report = diagnose_files(
            args.replay_pack,
            args.input_jsonl,
            args.output,
            thresholds,
        )
    except (OSError, DiagnosticError, parity.ParityAnalysisError) as error:
        print(f"x30 quadrangle match diagnostic failure: {error}", file=sys.stderr)
        return 1
    totals = report["totals"]
    print(
        "x30 quadrangle diagnostic: "
        f"frames={totals['frame_count']}, "
        f"candidates={totals['core_candidates']}, "
        f"factory_geometry={totals['factory_geometry_groups']}, "
        f"accepted_matches={totals['accepted_pairs']}, "
        f"pose_corrected_accepted_matches="
        f"{totals['pose_corrected_accepted_pairs']}, "
        f"cut_accepted_matches={totals['pre_merge_cut_accepted_pairs']}, "
        f"merged_pairs={totals['same_stair_merged_pairs']}, "
        f"merge_output={totals['same_stair_merge_output_candidates']}, "
        f"merge_accepted_matches={totals['same_stair_merge_accepted_pairs']}, "
        f"side_filtered_accepted_matches="
        f"{totals['side_distance_filtered_accepted_pairs']}, "
        f"merge_clues={totals['merge_clue_count']}, "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
