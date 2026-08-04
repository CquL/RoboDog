import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "summarize_x30_plane_seg_replay.py"
COMMITTED_FIXTURE_ROOT = (
    ROOT / "ws" / "src" / "x30_plane_seg_core" / "test" / "fixtures"
)
COMMITTED_PACK = COMMITTED_FIXTURE_ROOT / "x30_plane_seg_replay_v1.x30rpl"
COMMITTED_METRICS = COMMITTED_FIXTURE_ROOT / "x30_plane_seg_replay_v1.metrics.json"
SPEC = importlib.util.spec_from_file_location("x30_replay_summary", TOOL_PATH)
SUMMARY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SUMMARY)


def candidate(offset=0.0):
    return {
        "type": 0,
        "size": [1.0, 2.0, 0.142875],
        "translation": [offset, 0.0, -0.5],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "hull": [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
    }


def frame(case="flat", stamp_ns=100, selected_index=0, candidate_count=1):
    return {
        "case": case,
        "stamp_ns": stamp_ns,
        "selected_index": selected_index,
        "retained": 4093 + selected_index,
        "candidate_count": candidate_count,
        "factory_group_count": 1 + selected_index,
        "candidates": [candidate(float(index)) for index in range(candidate_count)],
    }


def jsonl(*frames):
    return b"".join(
        (json.dumps(item, separators=(",", ":"), sort_keys=False) + "\n").encode(
            "utf-8"
        )
        for item in frames
    )


class ReplaySummaryTest(unittest.TestCase):
    def test_normal_summary_and_canonical_frame_hash(self):
        second = frame("step", 200, 1, 2)
        first = frame("flat", 100, 0, 1)
        input_bytes = jsonl(second, first)

        result = SUMMARY.build_summary(input_bytes)

        self.assertEqual(result["schema"], "x30_plane_seg_replay_summary_v1")
        self.assertEqual(result["input_jsonl_sha256"], hashlib.sha256(input_bytes).hexdigest())
        self.assertEqual(result["frame_count"], 2)
        self.assertEqual(result["retained"], {"min": 4093, "max": 4094})
        self.assertEqual(
            result["candidates"],
            {"min": 1, "max": 2, "total": 3, "histogram": {"1": 1, "2": 1}},
        )
        self.assertEqual(
            result["factory_groups"], {"min": 1, "max": 2, "total": 3}
        )
        self.assertEqual(result["count_equal_factory"], 2)
        self.assertEqual(result["degenerate_candidate_count"], 0)
        self.assertEqual(result["degenerate_frames"], [])
        self.assertEqual([item["case"] for item in result["frames"]], ["flat", "step"])
        canonical = json.dumps(
            first,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
        self.assertEqual(
            result["frames"][0]["canonical_line_sha256"],
            hashlib.sha256(canonical).hexdigest(),
        )

    def test_duplicate_frame_key_is_rejected(self):
        duplicate = frame()
        with self.assertRaisesRegex(SUMMARY.SummaryError, "duplicate frame key"):
            SUMMARY.build_summary(jsonl(duplicate, duplicate))

    def test_candidate_count_mismatch_is_rejected(self):
        invalid = frame(candidate_count=1)
        invalid["candidate_count"] = 2
        with self.assertRaisesRegex(SUMMARY.SummaryError, "does not match"):
            SUMMARY.build_summary(jsonl(invalid))

    def test_degenerate_candidates_are_reported_without_filtering(self):
        valid = candidate()
        small_x = candidate(1.0)
        small_x["size"][0] = 1.0e-6
        small_y = candidate(2.0)
        small_y["size"][1] = 0.0
        short_hull = candidate(3.0)
        short_hull["hull"] = short_hull["hull"][:2]
        affected = frame("degenerate", 500, 4, 4)
        affected["candidates"] = [valid, small_x, small_y, short_hull]

        result = SUMMARY.build_summary(jsonl(affected))

        self.assertEqual(result["candidates"]["total"], 4)
        self.assertEqual(result["degenerate_candidate_count"], 3)
        self.assertEqual(
            result["degenerate_frames"],
            [
                {
                    "case": "degenerate",
                    "stamp_ns": 500,
                    "selected_index": 4,
                    "count": 3,
                }
            ],
        )

    def test_optional_contained_points_are_validated(self):
        extended = frame()
        extended["candidates"][0]["contained_points"] = [
            [0.1, 0.2, 0.3],
            [0.4, 0.5, 0.6],
        ]

        result = SUMMARY.build_summary(jsonl(extended))

        self.assertEqual(result["frame_count"], 1)
        self.assertEqual(result["candidates"]["total"], 1)

    def test_nonfinite_candidate_values_are_rejected(self):
        for token in ("NaN", "Infinity", "-Infinity", "1e400"):
            with self.subTest(token=token):
                raw = jsonl(frame()).decode("ascii")
                raw = raw.replace("0.142875", token, 1).encode("ascii")
                with self.assertRaisesRegex(SUMMARY.SummaryError, "finite"):
                    SUMMARY.build_summary(raw)

    def test_empty_and_blank_inputs_are_rejected(self):
        for input_bytes in (b"", b"\n"):
            with self.subTest(input_bytes=input_bytes):
                with self.assertRaises(SUMMARY.SummaryError):
                    SUMMARY.build_summary(input_bytes)

    def test_render_is_ascii_lf_and_deterministic(self):
        input_bytes = jsonl(frame("case-with-unicode-\u53f0\u9636", 300, 2, 0))
        first = SUMMARY.render_summary(SUMMARY.build_summary(input_bytes))
        second = SUMMARY.render_summary(SUMMARY.build_summary(input_bytes))

        self.assertEqual(first, second)
        first.decode("ascii")
        self.assertTrue(first.endswith(b"\n"))
        self.assertNotIn(b"\r", first)
        self.assertIn(b"\\u53f0\\u9636", first)

    def test_source_pack_hash_size_and_cli_output(self):
        input_bytes = jsonl(frame())
        pack_bytes = b"X30RPLY\x00" + bytes(range(32))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "replay.jsonl"
            pack_path = root / "fixture.x30rpl"
            output_path = root / "summary.json"
            input_path.write_bytes(input_bytes)
            pack_path.write_bytes(pack_bytes)

            completed = subprocess.run(
                [
                    sys.executable,
                    str(TOOL_PATH),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--source-pack",
                    str(pack_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(output_path.read_text(encoding="ascii"))
            self.assertEqual(
                result["source_pack"],
                {
                    "sha256": hashlib.sha256(pack_bytes).hexdigest(),
                    "size_bytes": len(pack_bytes),
                },
            )
            expected = SUMMARY.render_summary(
                SUMMARY.build_summary(input_bytes, pack_bytes)
            )
            self.assertEqual(output_path.read_bytes(), expected)

    def test_committed_metrics_are_bound_to_the_replay_pack(self):
        metrics = json.loads(COMMITTED_METRICS.read_text(encoding="ascii"))
        pack_bytes = COMMITTED_PACK.read_bytes()

        self.assertEqual(metrics["schema"], "x30_plane_seg_replay_summary_v1")
        self.assertEqual(metrics["frame_count"], 12)
        self.assertEqual(len(metrics["frames"]), 12)
        self.assertEqual(
            metrics["input_jsonl_sha256"],
            "42318c156d930052fb213e69078c66399dd806d673ef01ce6c10302ddbab909e",
        )
        self.assertEqual(
            metrics["source_pack"],
            {
                "sha256": hashlib.sha256(pack_bytes).hexdigest(),
                "size_bytes": len(pack_bytes),
            },
        )
        self.assertEqual(metrics["degenerate_candidate_count"], 0)
        self.assertEqual(metrics["degenerate_frames"], [])
        frame_keys = {
            (frame["case"], frame["stamp_ns"], frame["selected_index"])
            for frame in metrics["frames"]
        }
        self.assertEqual(len(frame_keys), 12)


if __name__ == "__main__":
    unittest.main()
