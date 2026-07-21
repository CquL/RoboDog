import importlib.util
import json
import struct
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock


TRANSFER_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = TRANSFER_ROOT / "tools"
MODULE_PATH = TOOLS_DIR / "export_x30_plane_seg_paired_fixtures.py"
PAIRED_FIXTURE_ROOT = (
    Path(__file__).resolve().parent / "fixtures" / "plane_seg_paired"
)
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import analyze_x30_gridmap_baseline as WIRE


SPEC = importlib.util.spec_from_file_location(
    "plane_seg_paired_fixture_exporter", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def stamp_parts(stamp_ns):
    return divmod(stamp_ns, 1_000_000_000)


def make_layer(name, raw_data, data_offset=0):
    values = struct.unpack(f"<{len(raw_data) // 4}f", raw_data)
    return WIRE.GridMapLayer(
        name=name,
        dimensions=(
            WIRE.MultiArrayDimension("column_index", 1, 2),
            WIRE.MultiArrayDimension("row_index", 2, 2),
        ),
        data_offset=data_offset,
        values=values,
        raw_data=raw_data,
    )


def make_grid(index, stamp_ns):
    sec, nsec = stamp_parts(stamp_ns)
    nan_payload = struct.pack("<I", 0x7FC01234)
    elevation_raw = nan_payload + struct.pack("<f", float(index))
    accessibility_raw = struct.pack("<2f", 1.0, index / 10.0)
    return MODULE.GridMapFrame(
        record_time_ns=stamp_ns + 10,
        sequence=index,
        stamp_sec=sec,
        stamp_nsec=nsec,
        frame_id="/world",
        resolution=0.5,
        length_x=0.5,
        length_y=1.0,
        position=(1.0, 2.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        layers=(
            make_layer("elevation", elevation_raw, data_offset=1),
            make_layer("accessibility", accessibility_raw),
        ),
        basic_layers=("elevation",),
        outer_start_index=index % 2,
        inner_start_index=(index + 1) % 2,
    )


def make_quadrangles(index, stamp_ns):
    sec, nsec = stamp_parts(stamp_ns)
    return MODULE.PointCloudFrame(
        record_time_ns=stamp_ns + 20,
        stamp_sec=sec,
        stamp_nsec=nsec,
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


def make_transform(stamp_ns, frame_id="/world", child_frame_id="/base_link"):
    sec, nsec = stamp_parts(stamp_ns)
    return MODULE.TransformStampedFrame(
        sequence=0,
        stamp_sec=sec,
        stamp_nsec=nsec,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        translation=(1.0, 2.0, 3.0),
        rotation=(0.0, 0.0, 0.0, 1.0),
    )


def make_inputs(complete_count=5, include_missing_first=True, include_orphan=True):
    first_complete_stamp = 20_000_000_100
    complete_stamps = [
        first_complete_stamp + index * 1_000_000_000
        for index in range(complete_count)
    ]
    missing_stamp = first_complete_stamp - 1_000_000_000
    grid_stamps = complete_stamps[:]
    if include_missing_first:
        grid_stamps.insert(0, missing_stamp)
    grids = tuple(make_grid(index, stamp) for index, stamp in enumerate(grid_stamps))

    quadrangle_stamps = grid_stamps[:]
    if include_orphan:
        quadrangle_stamps.insert(0, missing_stamp - 1_000_000_000)
    quadrangles = tuple(
        make_quadrangles(index, stamp)
        for index, stamp in enumerate(quadrangle_stamps)
    )
    dynamic_transforms = tuple(make_transform(stamp) for stamp in complete_stamps)
    if include_missing_first:
        dynamic_transforms += (
            make_transform(missing_stamp + 1),
        )
    tf_messages = (
        MODULE.TFMessageFrame(
            record_time_ns=30_000_000_000,
            topic="/tf",
            transforms=dynamic_transforms,
        ),
    )
    static_messages = (
        MODULE.TFMessageFrame(
            record_time_ns=1,
            topic="/tf_static",
            transforms=(make_transform(0, "/base_link", "/sensor"),),
        ),
    )
    look_poses = (
        WIRE.PoseFrame(
            record_time_ns=2,
            stamp_sec=0,
            stamp_nsec=0,
            frame_id="world",
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    return MODULE.RosbagPlaneSegInputs(
        grid_maps=grids,
        quadrangles=quadrangles,
        look_poses=look_poses,
        tf_messages=tf_messages,
        tf_static_messages=static_messages,
        height_map_mode_values=Counter({20: 1}),
        height_map_mode_state_values=Counter({3: len(grids)}),
    )


def directory_bytes(root):
    return {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


class PlaneSegPairedFixtureExporterTest(unittest.TestCase):
    def test_pairing_is_exact_and_records_orphan_and_missing_tf(self):
        inputs = make_inputs(complete_count=1)
        pairing = MODULE.pair_plane_seg_inputs(inputs)

        self.assertEqual(len(pairing.exact_pairs), 2)
        self.assertEqual(len(pairing.complete_pairs), 2)
        self.assertEqual(len(pairing.pairs_with_required_tf), 1)
        self.assertEqual(len(pairing.missing_required_tf), 1)
        self.assertEqual(len(pairing.orphan_quadrangles), 1)
        self.assertFalse(pairing.orphan_grid_maps)
        self.assertEqual(len(pairing.static_transforms), 1)
        missing = pairing.missing_required_tf[0]
        self.assertFalse(missing.has_required_tf)
        self.assertEqual(missing.exact_dynamic_transforms, ())
        self.assertEqual(inputs.look_poses[0].stamp_ns, 0)

    def test_all_and_zero_select_every_complete_pair(self):
        measured_step_like = make_inputs(
            complete_count=146,
            include_missing_first=True,
            include_orphan=False,
        )
        pairing = MODULE.pair_plane_seg_inputs(measured_step_like)
        self.assertEqual(len(pairing.complete_pairs), 147)
        self.assertEqual(len(pairing.pairs_with_required_tf), 146)
        self.assertEqual(len(pairing.missing_required_tf), 1)
        expected = list(range(147))
        self.assertEqual(
            MODULE.select_frame_indices(len(pairing.complete_pairs), "all"),
            expected,
        )
        self.assertEqual(
            MODULE.select_frame_indices(len(pairing.complete_pairs), 0),
            expected,
        )
        self.assertEqual(
            MODULE.select_frame_indices(len(pairing.complete_pairs), 3),
            [0, 73, 146],
        )
        self.assertEqual(MODULE.select_frame_indices(10, 3), [0, 4, 9])
        with self.assertRaisesRegex(ValueError, "non-negative"):
            MODULE.select_frame_indices(3, -1)

    def test_export_is_deterministic_and_preserves_raw_float32_bytes(self):
        inputs = make_inputs()
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            capture_dir = root / "captures"
            capture_dir.mkdir()
            bag_path = capture_dir / "synthetic.bag"
            bag_path.write_bytes(b"small synthetic bag")
            bag_hash = MODULE.sha256_file(bag_path)
            bag_path.with_suffix(".bag.sha256").write_text(
                f"{bag_hash}  /original/synthetic.bag\n", encoding="ascii"
            )
            bag_path.with_suffix(".metadata.txt").write_text(
                "label=synthetic\nrobot_motion_command_sent=false\n",
                encoding="utf-8",
            )
            bag_path.with_suffix(".info.yaml").write_text(
                "path: /original/synthetic.bag\n", encoding="utf-8"
            )

            with mock.patch.object(
                MODULE, "read_rosbag_plane_seg_inputs", return_value=inputs
            ) as reader:
                first_manifest = MODULE.export_fixtures(
                    [bag_path], root / "first", source_root=root
                )
                second_manifest = MODULE.export_fixtures(
                    [bag_path], root / "second", source_root=root
                )

            self.assertEqual(reader.call_count, 2)
            self.assertEqual(first_manifest, second_manifest)
            self.assertEqual(
                directory_bytes(root / "first"), directory_bytes(root / "second")
            )

            output = root / "first"
            fixture_path = output / "synthetic.plane_seg_paired.json"
            fixture = json.loads(fixture_path.read_bytes())
            self.assertEqual(fixture["schema"], MODULE.SCHEMA)
            self.assertEqual(fixture["source"]["bag_sha256"], bag_hash)
            self.assertEqual(
                fixture["source"]["declared_original_bag_path"],
                "/original/synthetic.bag",
            )
            for sidecar in fixture["provenance"]["sidecars"]:
                self.assertEqual(
                    MODULE.sha256_file(root / sidecar["path"]),
                    sidecar["sha256"],
                )
            self.assertEqual(len(fixture["frames"]), 3)
            self.assertEqual(
                fixture["provenance"]["selection"]["selected_complete_pair_indices"],
                [0, 2, 5],
            )
            self.assertEqual(
                fixture["provenance"]["selection"]["selected_pair_indices"],
                [0, 2, 5],
            )
            self.assertEqual(fixture["pairing"]["counts"]["missing_required_tf"], 1)
            self.assertEqual(fixture["pairing"]["counts"]["orphan_quadrangles"], 1)
            self.assertFalse(
                fixture["pairing"]["policy"]["nearest_or_future_tf_fallback"]
            )
            self.assertFalse(fixture["pairing"]["look_pose"]["used_as_pairing_key"])
            self.assertEqual(
                fixture["pairing"]["topic_values"]["/height_map_mode"][
                    "value_counts"
                ],
                {"20": 1},
            )
            self.assertEqual(
                fixture["pairing"]["topic_values"]["/height_map_mode_state"][
                    "value_counts"
                ],
                {"3": 6},
            )
            self.assertEqual(len(fixture["tf_static"]), 1)
            self.assertEqual(
                fixture["tf_static"][0]["header"]["frame_id"], "/base_link"
            )
            self.assertEqual(
                fixture["tf_static"][0]["normalized_frame_id"], "base_link"
            )

            first_frame = fixture["frames"][0]
            self.assertEqual(first_frame["source_frame_indices"]["grid_map"], 0)
            elevation = first_frame["grid_map"]["layers"][0]
            self.assertNotIn("values", elevation)
            self.assertEqual(elevation["data_offset"], 1)
            self.assertEqual(elevation["dimensions"][0]["stride"], 2)
            raw_blob = (output / elevation["blob_path"]).read_bytes()
            self.assertEqual(raw_blob[:4], struct.pack("<I", 0x7FC01234))
            self.assertEqual(MODULE.sha256_bytes(raw_blob), elevation["blob_sha256"])
            self.assertEqual(
                MODULE.sha256_bytes(
                    MODULE.canonical_json_bytes(
                        first_frame["expected_quadrangles"]
                    )
                ),
                first_frame["expected_quadrangles_sha256"],
            )
            self.assertEqual(first_frame["grid_map"]["outer_start_index"], 0)
            self.assertEqual(first_frame["grid_map"]["inner_start_index"], 1)
            self.assertEqual(
                first_frame["tf"]["status"],
                "missing_exact_required_dynamic_tf",
            )
            self.assertIsNone(first_frame["tf"]["world_to_base_link"])
            frame_with_tf = fixture["frames"][1]
            self.assertEqual(
                frame_with_tf["tf"]["world_to_base_link"]["header"]["stamp_ns"],
                frame_with_tf["grid_map"]["header"]["stamp_ns"],
            )

            manifest_bytes = (output / "manifest.json").read_bytes()
            declared_manifest_hash = (
                (output / "manifest.json.sha256")
                .read_text(encoding="ascii")
                .split()[0]
            )
            self.assertEqual(
                MODULE.sha256_bytes(manifest_bytes), declared_manifest_hash
            )
            manifest = json.loads(manifest_bytes)
            self.assertEqual(manifest["schema"], MODULE.MANIFEST_SCHEMA)
            entry = manifest["fixtures"][0]
            self.assertEqual(entry["fixture_sha256"], MODULE.sha256_file(fixture_path))
            self.assertEqual(len(entry["blobs"]), 6)
            for blob in entry["blobs"]:
                blob_path = output / blob["path"]
                self.assertEqual(blob_path.stat().st_size, blob["size_bytes"])
                self.assertEqual(MODULE.sha256_file(blob_path), blob["sha256"])

    def test_committed_real_fixtures_are_hash_verified_and_auditable(self):
        manifest_path = PAIRED_FIXTURE_ROOT / "manifest.json"
        manifest_bytes = manifest_path.read_bytes()
        declared_manifest_hash = (
            (PAIRED_FIXTURE_ROOT / "manifest.json.sha256")
            .read_text(encoding="ascii")
            .split()[0]
        )
        self.assertEqual(
            MODULE.sha256_bytes(manifest_bytes), declared_manifest_hash
        )

        manifest = json.loads(manifest_bytes)
        self.assertEqual(manifest["schema"], MODULE.MANIFEST_SCHEMA)
        self.assertEqual(len(manifest["fixtures"]), 4)
        self.assertFalse(list(PAIRED_FIXTURE_ROOT.rglob("*.bag")))

        expected_counts = {
            "mode3_flat_20260714_142726.bag": (141, 142, 141, 141, 0, 1),
            "mode3_flat_repeat_20260714_150324.bag": (
                142,
                142,
                142,
                142,
                0,
                0,
            ),
            "mode3_measured_step_h022_d029_w035_20260714_160857.bag": (
                147,
                147,
                147,
                146,
                1,
                0,
            ),
            "mode3_object_probe_20260714_153211.bag": (
                141,
                141,
                141,
                141,
                0,
                0,
            ),
        }
        total_frames = 0
        total_blob_bytes = 0
        for entry in manifest["fixtures"]:
            fixture_path = PAIRED_FIXTURE_ROOT / entry["fixture_path"]
            fixture_bytes = fixture_path.read_bytes()
            self.assertEqual(len(fixture_bytes), entry["fixture_size_bytes"])
            self.assertEqual(
                MODULE.sha256_bytes(fixture_bytes), entry["fixture_sha256"]
            )

            fixture = json.loads(fixture_bytes)
            bag_name = Path(fixture["source"]["bag_path"]).name
            counts = fixture["pairing"]["counts"]
            actual_counts = (
                counts["grid_maps"],
                counts["quadrangles"],
                counts["exact_grid_map_quadrangles_pairs"],
                counts["pairs_with_required_tf"],
                counts["missing_required_tf"],
                counts["orphan_quadrangles"],
            )
            self.assertEqual(actual_counts, expected_counts[bag_name])
            self.assertFalse(
                fixture["pairing"]["policy"]["nearest_or_future_tf_fallback"]
            )
            self.assertEqual(len(fixture["frames"]), 3)
            total_frames += len(fixture["frames"])

            for frame in fixture["frames"]:
                grid_stamp = frame["grid_map"]["header"]["stamp_ns"]
                quadrangle_stamp = frame["expected_quadrangles"]["header"][
                    "stamp_ns"
                ]
                self.assertEqual(grid_stamp, quadrangle_stamp)
                points = frame["expected_quadrangles"]["points_xyz"]
                self.assertEqual(len(points) % 4, 0)
                # Factory output consistently retains this degenerate first group.
                self.assertEqual(points[2], points[3])

                layers = {
                    layer["name"]: layer
                    for layer in frame["grid_map"]["layers"]
                }
                self.assertIn("elevation", layers)
                self.assertIn("accessibility", layers)
                for layer in layers.values():
                    self.assertEqual(layer["value_count"], 10_000)
                    self.assertEqual(layer["blob_size_bytes"], 40_000)
                    blob_path = PAIRED_FIXTURE_ROOT / layer["blob_path"]
                    self.assertEqual(
                        MODULE.sha256_file(blob_path), layer["blob_sha256"]
                    )

                required_tf = frame["tf"]["world_to_base_link"]
                if required_tf is not None:
                    self.assertEqual(
                        required_tf["header"]["stamp_ns"], grid_stamp
                    )

            for blob in entry["blobs"]:
                blob_path = PAIRED_FIXTURE_ROOT / blob["path"]
                self.assertEqual(blob_path.stat().st_size, blob["size_bytes"])
                self.assertEqual(MODULE.sha256_file(blob_path), blob["sha256"])
                total_blob_bytes += blob["size_bytes"]

        self.assertEqual(total_frames, 12)
        self.assertEqual(total_blob_bytes, 1_920_000)


if __name__ == "__main__":
    unittest.main()
