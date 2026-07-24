#!/usr/bin/env python3
"""Tests for the offline candidate/factory quadrangle diagnostic."""

from __future__ import annotations

import json
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import analyze_x30_quadrangle_parity as parity  # noqa: E402
import diagnose_x30_quadrangle_matches as diagnostic  # noqa: E402


def factory_sentinel(z_min: float = 0.0):
    return (
        (1.001, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (-1.0, 0.0, z_min + 0.07),
        (-1.0, 0.0, z_min + 0.07),
    )


def rectangle(
    center_x: float,
    width: float,
    depth: float,
    z: float = 0.0,
    center_y: float = 0.0,
):
    half_x = width * 0.5
    half_y = depth * 0.5
    return (
        (center_x - half_x, center_y - half_y, z),
        (center_x - half_x, center_y + half_y, z),
        (center_x + half_x, center_y + half_y, z),
        (center_x + half_x, center_y - half_y, z),
    )


def candidate(
    center_x: float,
    width: float,
    depth: float,
    z: float = 0.0,
    center_y: float = 0.0,
    with_contained_points: bool = False,
):
    size_z = 0.2
    hull = rectangle(center_x, width, depth, z, center_y)
    return parity.Candidate(
        type=0,
        size=(width, depth, size_z),
        translation=(center_x, center_y, z - size_z * 0.5),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        hull=hull,
        contained_points=hull if with_contained_points else (),
    )


def replay_frame(selected_index, stamp_ns, factory_points):
    return parity.ReplayFrame(
        case="synthetic_measured_step",
        stamp_ns=stamp_ns,
        selected_index=selected_index,
        expected_retained_count=42,
        factory_points=tuple(factory_points),
    )


def output_frame(frame, candidates):
    return parity.ReplayOutputFrame(
        case=frame.case,
        stamp_ns=frame.stamp_ns,
        selected_index=frame.selected_index,
        retained=frame.expected_retained_count,
        factory_group_count=len(frame.factory_points) // 4,
        candidates=tuple(candidates),
    )


class CandidateTopRectangleTest(unittest.TestCase):
    def test_empty_repair_boundary_uses_factory_asymmetric_x_limits(self):
        self.assertEqual(
            diagnostic._compute_repair_x_bounds([], 2.0, 5.0),
            (6.0, -4.0),
        )

    def test_factory_sentinel_decodes_staged_tail_count(self):
        self.assertEqual(
            diagnostic._factory_staged_tail_count(
                (factory_sentinel(),), True, 1.0e-5
            ),
            1,
        )
        self.assertIsNone(
            diagnostic._factory_staged_tail_count((), False, 1.0e-5)
        )

    def test_factory_wide_height_gate_excludes_exact_tolerance(self):
        self.assertTrue(
            diagnostic._factory_wide_height_eligible(0.0, 0.249, 0.25)
        )
        self.assertFalse(
            diagnostic._factory_wide_height_eligible(0.0, 0.25, 0.25)
        )

    def test_contained_point_cut_quantizes_factory_bounds_to_float32(self):
        result = diagnostic._cut_repair_hypothesis_by_contained_points(
            [(0.699999995, 0.5)],
            y_norm=1.0,
            x_max=0.7,
            x_min=0.0,
            expand_to_08=False,
        )

        self.assertEqual(result["score"], 0)

    def test_matches_cpp_half_height_and_clockwise_contract(self):
        top = diagnostic.candidate_top_rectangle(candidate(0.25, 2.0, 1.0), 7)

        self.assertTrue(top.success, top.diagnostic)
        self.assertEqual(top.candidate_index, 7)
        self.assertIsNotNone(top.rectangle)
        self.assertAlmostEqual(top.rectangle.center[0], 0.25)
        self.assertAlmostEqual(top.rectangle.center[2], 0.0)
        self.assertAlmostEqual(top.rectangle.area_3d_m2, 2.0)
        self.assertLess(top.rectangle.signed_xy_area_m2, 0.0)

    def test_rejects_frame_132_empty_zero_size_signature(self):
        degenerate = parity.Candidate(
            type=0,
            size=(0.0, 0.0, 0.142875001),
            translation=(0.0, 0.0, -0.0714375004),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            hull=(),
        )
        top = diagnostic.candidate_top_rectangle(degenerate, 0)

        self.assertFalse(top.success)
        self.assertEqual(top.diagnostic, "candidate has a degenerate size")

    def test_pose_correction_recovers_consensus_with_hull_proxy(self):
        candidates = [
            candidate(
                center_x=-0.8 + index * 0.4,
                width=0.3,
                depth=1.0,
                z=-0.7 + index * 0.18,
            )
            for index in range(5)
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(candidates)
        ]

        corrected, report = diagnostic.correct_candidate_top_rectangles(tops)

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["eligible_stair_pose_count"], 5)
        self.assertEqual(report["consensus_direction_count"], 5)
        self.assertEqual(len(corrected), 5)
        direction = report["corrected_stair_direction_xy"]
        self.assertAlmostEqual(
            math.hypot(direction[0], direction[1]), 1.0, places=9
        )
        self.assertGreater(abs(direction[0]), 0.999)

    def test_replay_parser_accepts_optional_contained_points(self):
        raw = {
            "case": "extended",
            "stamp_ns": 1,
            "selected_index": 0,
            "retained": 1,
            "candidate_count": 1,
            "factory_group_count": 0,
            "candidates": [
                {
                    "type": 0,
                    "size": [0.3, 1.0, 0.2],
                    "translation": [0.0, 0.0, -0.1],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "hull": [list(point) for point in rectangle(0.0, 0.3, 1.0)],
                    "contained_points": [[0.0, 0.0, 0.0]],
                }
            ],
        }

        frames = parity.parse_replay_jsonl(
            (json.dumps(raw, separators=(",", ":")) + "\n").encode("ascii")
        )

        self.assertEqual(frames[0].candidates[0].contained_points, ((0.0, 0.0, 0.0),))

    def test_side_distance_filter_chooses_axis_with_fewer_removals(self):
        inputs = [
            candidate(1.0, 0.2, 0.2, center_y=0.0),
            candidate(1.0, 0.2, 0.2, center_y=0.6),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        filtered, report = diagnostic.ignore_unnecessary_candidate_tops(
            tops, (1.0, 0.0)
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["original_axis_removal_count"], 1)
        self.assertEqual(report["perpendicular_axis_removal_count"], 2)
        self.assertEqual(report["removed_vector_indices"], [1])
        self.assertEqual(report["removed_candidate_indices"], [1])
        self.assertEqual([item.candidate_index for item in filtered], [0])

    def test_side_distance_filter_erases_by_current_vector_position(self):
        near = diagnostic.candidate_top_rectangle(
            candidate(1.0, 0.2, 0.2, center_y=0.0), 7
        )
        far = replace(
            diagnostic.candidate_top_rectangle(
                candidate(1.0, 0.2, 0.2, center_y=0.6), 8
            ),
            candidate_index=7,
        )

        filtered, report = diagnostic.ignore_unnecessary_candidate_tops(
            [near, far], (1.0, 0.0)
        )

        self.assertEqual(report["removed_vector_indices"], [1])
        self.assertEqual([item.candidate_index for item in filtered], [7])

    def test_side_distance_filter_preserves_preexisting_invalid_candidate(self):
        valid = diagnostic.candidate_top_rectangle(
            candidate(1.0, 0.2, 0.2, center_y=0.0), 0
        )
        degenerate = parity.Candidate(
            type=0,
            size=(0.0, 0.0, 0.2),
            translation=(0.0, 0.0, -0.1),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            hull=(),
        )
        invalid = diagnostic.candidate_top_rectangle(degenerate, 1)

        filtered, report = diagnostic.ignore_unnecessary_candidate_tops(
            [valid, invalid], (1.0, 0.0)
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual([item.candidate_index for item in filtered], [0, 1])
        self.assertFalse(filtered[1].success)

    def test_intrusion_classification_sorts_and_splits_factory_groups(self):
        inputs = [
            candidate(0.0, 0.4, 0.1),
            candidate(0.0, 0.8, 0.6),
            candidate(0.0, 1.0, 0.2),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        report = diagnostic.classify_candidate_intrusions(tops, 0.05)

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["sorted_vector_indices"], [2, 0, 1])
        self.assertEqual(report["non_intruding_vector_indices"], [2, 0])
        self.assertEqual(report["intruded_short_edge_vector_indices"], [])
        self.assertEqual(report["intruded_long_edge_vector_indices"], [1])

    def test_reference_selection_defers_then_resolves_adjacent_sides(self):
        inputs = [
            candidate(-0.25, 0.2, 0.2, z=0.0),
            candidate(0.0, 0.2, 0.2, z=0.5),
            candidate(0.25, 0.2, 0.2, z=1.0),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        first_pass = diagnostic.select_candidate_reference(
            tops, [0, 2], 1, 0.03, (0.0, 0.0), False
        )
        fallback = diagnostic.select_candidate_reference(
            tops, [0, 2], 1, 0.03, (0.0, 0.0), True
        )

        self.assertTrue(first_pass["success"], first_pass["diagnostic"])
        self.assertTrue(first_pass["deferred_for_fallback"])
        self.assertEqual(first_pass["selected_reference_vector_index"], -1)
        self.assertTrue(fallback["success"], fallback["diagnostic"])
        self.assertEqual(fallback["selected_reference_vector_index"], 0)
        self.assertEqual(len(fallback["constraint_points_xyz"]), 4)

    def test_reference_selection_promotes_unrelated_target(self):
        inputs = [
            candidate(-2.0, 0.2, 0.2, z=0.0),
            candidate(0.0, 0.2, 0.2, z=0.5),
            candidate(2.0, 0.2, 0.2, z=1.0),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        report = diagnostic.select_candidate_reference(
            tops, [0, 2], 1, 0.03, (0.0, 0.0), False
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertTrue(report["target_added_to_reference_indices"])
        self.assertEqual(report["updated_reference_vector_indices"], [0, 2, 1])

    def test_reference_repair_commits_earliest_best_hypothesis(self):
        inputs = [
            candidate(
                0.0, 1.0, 0.1, z=0.0, center_y=0.0,
                with_contained_points=True,
            ),
            candidate(
                0.0, 0.8, 0.2, z=0.2, center_y=-0.35,
                with_contained_points=True,
            ),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        updated, report = (
            diagnostic.compute_and_update_candidate_quadrangles_from_reference(
                tops,
                [0],
                destination_vector_index=1,
                store_vector_index=1,
                reference_vector_index=0,
                grid_resolution_m=0.03,
                emit_all=False,
            )
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["emitted_quadrangle_count"], 1)
        self.assertEqual(report["registered_vector_indices"], [1])
        self.assertEqual(len(updated), 2)
        self.assertEqual(updated[1].candidate_index, 1)
        self.assertAlmostEqual(updated[1].rectangle.center[2], 0.2)
        self.assertEqual(
            updated[1].contained_points,
            tops[1].contained_points,
        )

    def test_reference_repair_emit_all_can_append(self):
        inputs = [
            candidate(
                0.0, 1.0, 0.1, z=0.0,
                with_contained_points=True,
            ),
            candidate(
                0.0, 0.8, 0.2, z=0.2, center_y=-0.35,
                with_contained_points=True,
            ),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        updated, report = (
            diagnostic.compute_and_update_candidate_quadrangles_from_reference(
                tops,
                [0],
                destination_vector_index=1,
                store_vector_index=len(tops),
                reference_vector_index=0,
                grid_resolution_m=0.03,
                emit_all=True,
            )
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertGreaterEqual(report["emitted_quadrangle_count"], 1)
        self.assertEqual(
            len(updated),
            len(tops) + report["emitted_quadrangle_count"],
        )
        self.assertEqual(
            report["registered_vector_indices"][0],
            len(tops),
        )

    def test_reference_repair_reports_empty_contained_points(self):
        inputs = [
            candidate(0.0, 1.0, 0.1, z=0.0),
            candidate(0.0, 0.8, 0.2, z=0.2, center_y=-0.35),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        _, report = (
            diagnostic.compute_and_update_candidate_quadrangles_from_reference(
                tops,
                [0],
                destination_vector_index=1,
                store_vector_index=1,
                reference_vector_index=0,
                grid_resolution_m=0.03,
                emit_all=False,
            )
        )

        self.assertFalse(report["success"])
        self.assertEqual(report["error_vector_indices"], [1])
        self.assertIn("contained-point cloud is empty", report["diagnostic"])

    def test_factory_final_suffix_repairs_intruded_short_target(self):
        inputs = [
            candidate(
                0.0, 1.0, 0.1, z=0.0,
                with_contained_points=True,
            ),
            candidate(
                0.0, 1.0, 0.2, z=0.2, center_y=-0.05,
                with_contained_points=True,
            ),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        final, report = diagnostic.run_factory_final_suffix(
            tops,
            grid_resolution_m=0.03,
            grid_map_position_xy=(0.0, 0.0),
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(
            report["classification"]["intruded_short_edge_vector_indices"],
            [1],
        )
        self.assertEqual(report["narrow_normal_attempt_count"], 1)
        self.assertEqual(report["narrow_normal_success_count"], 1)
        self.assertEqual(report["deleted_error_quadrangle_count"], 0)
        self.assertEqual(report["final_candidate_count"], len(final))
        self.assertEqual(
            [item.candidate_index for item in final],
            list(range(len(final))),
        )

    def test_factory_sampled_distance_and_same_stair_merge(self):
        inputs = [
            candidate(0.0, 1.0, 0.2, z=-0.5, center_y=-0.125),
            candidate(0.0, 1.0, 0.2, z=-0.5, center_y=0.125),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        distance = diagnostic.compute_candidate_distance_vector_xy(
            tops[0], tops[1]
        )
        self.assertAlmostEqual(math.hypot(*distance), 0.05, places=8)

        merged, report = diagnostic.merge_candidate_tops_at_same_stair(
            tops, (1.0, 0.0)
        )
        valid = [item for item in merged if item.success]

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["merged_pair_count"], 1)
        self.assertEqual(report["output_candidate_count"], 1)
        self.assertEqual(len(valid), 1)
        self.assertEqual(
            report["merged_source_candidate_indices"], [[0, 1]]
        )

    def test_factory_pcl_intrusion_accepts_rotated_boundary_point(self):
        angle = math.radians(27.0)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        source = rectangle(0.4, 0.8, 0.3, center_y=-0.2)
        rotated = tuple(
            (
                cosine * point[0] - sine * point[1],
                sine * point[0] + cosine * point[1],
                point[2],
            )
            for point in source
        )
        target = diagnostic.Rectangle(
            points=rotated,
            center=(
                sum(point[0] for point in rotated) / 4.0,
                sum(point[1] for point in rotated) / 4.0,
                0.0,
            ),
            area_3d_m2=0.24,
            signed_xy_area_m2=-0.24,
        )

        self.assertTrue(
            diagnostic._factory_pcl_boundary_intrudes(
                target,
                [(rotated[0][0], rotated[0][1])],
            )
        )

    def test_factory_pcl_intrusion_rejects_point_beyond_local_limit(self):
        target_points = rectangle(0.0, 1.0, 0.4)
        target = diagnostic.Rectangle(
            points=target_points,
            center=(0.0, 0.0, 0.0),
            area_3d_m2=0.4,
            signed_xy_area_m2=-0.4,
        )

        self.assertFalse(
            diagnostic._factory_pcl_boundary_intrudes(
                target,
                [(0.5001, 0.0)],
            )
        )

    def test_same_stair_merge_keeps_initial_height_gate_for_chain(self):
        inputs = [
            candidate(0.0, 1.0, 0.2, z=0.00, center_y=-0.20),
            candidate(0.0, 1.0, 0.2, z=0.06, center_y=0.05),
            candidate(0.0, 1.0, 0.2, z=0.09, center_y=0.30),
        ]
        tops = [
            diagnostic.candidate_top_rectangle(value, index)
            for index, value in enumerate(inputs)
        ]

        merged, report = diagnostic.merge_candidate_tops_at_same_stair(
            tops, (1.0, 0.0)
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["merged_pair_count"], 1)
        self.assertEqual(
            len([item for item in merged if item.success]), 2
        )

    def test_cut_by_x_is_deterministic_and_preserves_point_count(self):
        source = candidate(0.0, 1.0, 0.4, z=-0.5)
        top = diagnostic.candidate_top_rectangle(source, 5)

        cut, report = diagnostic.cut_candidate_tops_by_x(
            [top], (1.0, 0.0)
        )

        self.assertTrue(report["success"], report["diagnostic"])
        self.assertEqual(report["processed_candidate_count"], 1)
        self.assertEqual(len(cut), 1)
        self.assertEqual(
            len(cut[0].contained_points),
            len(source.hull),
        )


class DiagnosticReportTest(unittest.TestCase):
    def make_pack_and_outputs(self):
        first_factory = (*factory_sentinel(), *rectangle(0.0, 2.0, 1.0))
        first = replay_frame(7, 7_000, first_factory)
        first_output = output_frame(
            first,
            (
                candidate(-0.5, 1.0, 1.0),
                candidate(0.5, 1.0, 1.0),
            ),
        )

        frame_132_factory = (*factory_sentinel(), *rectangle(0.0, 1.0, 1.0))
        frame_132 = replay_frame(
            132,
            diagnostic.KNOWN_FRAME_132_STAMP_NS,
            frame_132_factory,
        )
        frame_132_output = output_frame(
            frame_132,
            (
                parity.Candidate(
                    type=0,
                    size=(0.0, 0.0, 0.142875001),
                    translation=(0.0, 0.0, -0.0714375004),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    hull=(),
                ),
                candidate(0.0, 1.0, 1.0),
            ),
        )
        pack = parity.ReplayPack(
            header=parity.ReplayHeader(
                source_manifest_sha256="11" * 32,
                body_sha256="22" * 32,
                absolute_tolerance=1.0e-5,
                relative_tolerance=1.0e-5,
            ),
            sha256="33" * 32,
            size_bytes=1234,
            frames=(first, frame_132),
        )
        return pack, (first_output, frame_132_output)

    def test_reports_count_geometry_merge_clues_and_frame_132(self):
        pack, outputs = self.make_pack_and_outputs()
        thresholds = diagnostic.Thresholds(center_xy_m=0.60)
        report = diagnostic.build_diagnostic_from_parsed(
            pack,
            outputs,
            thresholds,
            replay_jsonl_sha256="44" * 32,
            replay_jsonl_size_bytes=5678,
        )

        self.assertEqual(report["schema"], diagnostic.SCHEMA_NAME)
        self.assertEqual(report["totals"]["frame_count"], 2)
        self.assertEqual(report["totals"]["factory_groups_raw"], 4)
        self.assertEqual(report["totals"]["factory_geometry_groups"], 2)
        self.assertEqual(report["totals"]["core_candidates"], 4)
        self.assertEqual(report["totals"]["pose_correction_success_frames"], 0)

        first = report["frames"][0]
        self.assertTrue(first["counts"]["factory_sentinel_present"])
        self.assertEqual(first["counts"]["factory_staged_tail_count"], 1)
        self.assertEqual(
            first["factory_final_suffix"]["factory_staged_tail_count"], 1
        )
        self.assertEqual(first["counts"]["candidate_minus_factory_geometry"], 1)
        self.assertEqual(first["matching"]["matched_pair_count"], 1)
        self.assertGreaterEqual(len(first["merge_clues"]), 1)
        self.assertEqual(first["merge_clues"][0]["candidate_indices"], [0, 1])
        self.assertEqual(first["merge_clues"][0]["factory_group_index"], 1)

        summary = report["frame_132_summary"]
        self.assertEqual(summary["target_frame_count"], 1)
        self.assertEqual(summary["targets"][0]["degenerate_candidate_count"], 1)
        self.assertTrue(
            summary["targets"][0]["degenerate_candidates"][0][
                "matches_known_empty_zero_size_signature"
            ]
        )

        rendered = diagnostic.render_diagnostic(report)
        decoded = json.loads(rendered)
        self.assertEqual(decoded["schema"], diagnostic.SCHEMA_NAME)
        self.assertNotIn(b"NaN", rendered)

    def test_committed_replay_pack_uses_existing_contract(self):
        pack_path = (
            ROOT
            / "ws"
            / "src"
            / "x30_plane_seg_core"
            / "test"
            / "fixtures"
            / "x30_plane_seg_replay_v1.x30rpl"
        )
        pack_bytes = pack_path.read_bytes()
        pack = parity.parse_replay_pack(pack_bytes)
        lines = []
        for frame in pack.frames:
            lines.append(
                json.dumps(
                    {
                        "case": frame.case,
                        "stamp_ns": frame.stamp_ns,
                        "selected_index": frame.selected_index,
                        "retained": frame.expected_retained_count,
                        "candidate_count": 0,
                        "factory_group_count": len(frame.factory_points) // 4,
                        "candidates": [],
                    },
                    separators=(",", ":"),
                )
            )
        report = diagnostic.build_diagnostic(
            pack_bytes,
            ("\n".join(lines) + "\n").encode("ascii"),
        )

        self.assertEqual(report["totals"]["frame_count"], 12)
        self.assertEqual(report["totals"]["factory_groups_raw"], 83)
        self.assertEqual(report["totals"]["core_candidates"], 0)


if __name__ == "__main__":
    unittest.main()
