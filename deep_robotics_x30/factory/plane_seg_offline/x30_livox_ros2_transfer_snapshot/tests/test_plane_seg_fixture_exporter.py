import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


TRANSFER_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = TRANSFER_ROOT / "tools"
MODULE_PATH = TOOLS_DIR / "export_x30_plane_seg_fixtures.py"
sys.path.insert(0, str(TOOLS_DIR))
SPEC = importlib.util.spec_from_file_location("plane_seg_fixture_exporter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

FIXTURE_ROOT = Path(__file__).resolve().parent / "fixtures" / "plane_seg"


def make_frame(index: int) -> MODULE.PointCloudFrame:
    return MODULE.PointCloudFrame(
        record_time_ns=20_000_000_000 + index,
        stamp_sec=10 + index,
        stamp_nsec=100 + index,
        frame_id="world",
        width=4,
        height=1,
        points=(
            (float(index), 0.0, 0.0),
            (float(index), 1.0, 0.0),
            (float(index), 1.0, 1.0),
            (float(index), 0.0, 1.0),
        ),
    )


class PlaneSegFixtureExporterTest(unittest.TestCase):
    def test_select_frame_indices_is_even_and_inclusive(self):
        self.assertEqual(MODULE.select_frame_indices(10, 3), [0, 4, 9])
        self.assertEqual(MODULE.select_frame_indices(10, 1), [5])
        self.assertEqual(MODULE.select_frame_indices(2, 4), [0, 1])
        with self.assertRaisesRegex(ValueError, "empty topic"):
            MODULE.select_frame_indices(0, 3)
        with self.assertRaisesRegex(ValueError, "at least one"):
            MODULE.select_frame_indices(3, 0)

    def test_export_is_byte_for_byte_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_dir = root / "captures"
            source_dir.mkdir()
            bag_path = source_dir / "sample.bag"
            bag_path.write_bytes(b"small synthetic bag input")
            bag_sha256 = MODULE.sha256_file(bag_path)
            bag_path.with_suffix(".bag.sha256").write_text(
                f"{bag_sha256}  /capture/original_sample.bag\n",
                encoding="ascii",
            )
            bag_path.with_suffix(".metadata.txt").write_text(
                "label=synthetic\nrobot_motion_command_sent=false\n",
                encoding="utf-8",
            )
            bag_path.with_suffix(".info.yaml").write_text(
                "path: original_sample.bag\n",
                encoding="utf-8",
            )
            frames = [make_frame(index) for index in range(5)]

            with mock.patch.object(
                MODULE,
                "read_rosbag_quadrangles",
                return_value=(frames, {}, []),
            ) as reader:
                first_manifest = MODULE.export_fixtures(
                    [bag_path], root / "first", source_root=root
                )
                second_manifest = MODULE.export_fixtures(
                    [bag_path], root / "second", source_root=root
                )

            self.assertEqual(reader.call_count, 2)
            self.assertEqual(first_manifest, second_manifest)
            first_files = {
                path.name: path.read_bytes()
                for path in (root / "first").iterdir()
            }
            second_files = {
                path.name: path.read_bytes()
                for path in (root / "second").iterdir()
            }
            self.assertEqual(first_files, second_files)

            fixture = json.loads(first_files["sample.plane_seg.json"])
            self.assertEqual(fixture["source"]["bag_path"], "captures/sample.bag")
            self.assertEqual(fixture["source"]["bag_sha256"], bag_sha256)
            self.assertEqual(
                fixture["source"]["declared_original_bag_path"],
                "/capture/original_sample.bag",
            )
            self.assertEqual(
                fixture["source"]["capture_metadata"]["label"], "synthetic"
            )
            self.assertEqual(
                fixture["provenance"]["selection"]["selected_frame_indices"],
                [0, 2, 4],
            )
            for frame in fixture["frames"]:
                self.assertEqual(
                    frame["payload_sha256"],
                    MODULE.sha256_bytes(
                        MODULE.canonical_json_bytes(frame["payload"])
                    ),
                )

    def test_export_rejects_a_stale_source_hash_sidecar(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            bag_path = root / "sample.bag"
            bag_path.write_bytes(b"changed bag")
            bag_path.with_suffix(".bag.sha256").write_text(
                f"{'0' * 64}  sample.bag\n", encoding="ascii"
            )
            with mock.patch.object(
                MODULE,
                "read_rosbag_quadrangles",
                return_value=([make_frame(0)], {}, []),
            ):
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    MODULE.export_fixtures(
                        [bag_path], root / "output", source_root=root
                    )

    def test_committed_fixtures_are_compact_and_hash_verified(self):
        manifest_path = FIXTURE_ROOT / "manifest.json"
        manifest_bytes = manifest_path.read_bytes()
        declared_manifest_hash = (
            (FIXTURE_ROOT / "manifest.json.sha256")
            .read_text(encoding="ascii")
            .split()[0]
        )
        self.assertEqual(MODULE.sha256_bytes(manifest_bytes), declared_manifest_hash)

        manifest = json.loads(manifest_bytes)
        self.assertEqual(manifest["schema"], MODULE.MANIFEST_SCHEMA)
        self.assertEqual(len(manifest["fixtures"]), 4)
        self.assertFalse(list(FIXTURE_ROOT.rglob("*.bag")))

        expected_indices = {
            "mode3_flat_20260714_142726.plane_seg.json": [0, 70, 141],
            "mode3_flat_repeat_20260714_150324.plane_seg.json": [0, 70, 141],
            "mode3_measured_step_h022_d029_w035_20260714_160857.plane_seg.json": [
                0,
                73,
                146,
            ],
            "mode3_object_probe_20260714_153211.plane_seg.json": [0, 70, 140],
        }
        total_fixture_bytes = 0
        for entry in manifest["fixtures"]:
            fixture_path = FIXTURE_ROOT / entry["fixture_path"]
            fixture_bytes = fixture_path.read_bytes()
            total_fixture_bytes += len(fixture_bytes)
            self.assertEqual(len(fixture_bytes), entry["fixture_size_bytes"])
            self.assertEqual(
                MODULE.sha256_bytes(fixture_bytes), entry["fixture_sha256"]
            )

            fixture = json.loads(fixture_bytes)
            self.assertEqual(fixture["schema"], MODULE.SCHEMA)
            self.assertEqual(
                fixture["provenance"]["topic"], MODULE.DEFAULT_TOPIC
            )
            self.assertEqual(
                fixture["provenance"]["selection"]["selected_frame_indices"],
                expected_indices[entry["fixture_path"]],
            )
            self.assertEqual(len(fixture["frames"]), 3)
            self.assertGreater(fixture["source"]["bag_size_bytes"], len(fixture_bytes))
            self.assertRegex(fixture["source"]["bag_sha256"], r"^[0-9a-f]{64}$")

            payload_hashes = []
            for frame in fixture["frames"]:
                payload = frame["payload"]
                self.assertEqual(payload["header"]["frame_id"], "world")
                self.assertEqual(
                    payload["shape"]["point_count"], len(payload["points_xyz"])
                )
                self.assertEqual(
                    payload["shape"]["point_count"],
                    payload["shape"]["height"] * payload["shape"]["width"],
                )
                payload_hash = MODULE.sha256_bytes(
                    MODULE.canonical_json_bytes(payload)
                )
                self.assertEqual(frame["payload_sha256"], payload_hash)
                payload_hashes.append(payload_hash)
            self.assertEqual(entry["frame_payload_sha256"], payload_hashes)

        self.assertLess(total_fixture_bytes, 250_000)


if __name__ == "__main__":
    unittest.main()
