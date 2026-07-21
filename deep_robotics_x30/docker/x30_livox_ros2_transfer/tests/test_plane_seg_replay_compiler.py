import hashlib
import importlib.util
import json
import math
import struct
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path


TRANSFER_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = TRANSFER_ROOT / "tools" / "compile_x30_plane_seg_replay.py"
PAIRED_FIXTURE_ROOT = Path(__file__).resolve().parent / "fixtures" / "plane_seg_paired"
PACK_PATH = (
    TRANSFER_ROOT
    / "ws"
    / "src"
    / "x30_plane_seg_core"
    / "test"
    / "fixtures"
    / "x30_plane_seg_replay_v1.x30rpl"
)

SPEC = importlib.util.spec_from_file_location("plane_seg_replay_compiler", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def formatted_json_bytes(value):
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


def canonical_json_bytes(value):
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def stamp_header(stamp_ns, frame_id="world"):
    stamp_sec, stamp_nsec = divmod(stamp_ns, 1_000_000_000)
    return {
        "frame_id": frame_id,
        "stamp_ns": stamp_ns,
        "stamp_nsec": stamp_nsec,
        "stamp_sec": stamp_sec,
    }


def exact_transform(stamp_ns):
    return {
        "child_frame_id": "base_link",
        "header": stamp_header(stamp_ns),
        "normalized_child_frame_id": "base_link",
        "normalized_frame_id": "world",
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "translation_xyz": [1.0, 2.0, 3.0],
    }


def layer_metadata(name, path, raw):
    return {
        "blob_path": path,
        "blob_sha256": sha256(raw),
        "blob_size_bytes": len(raw),
        "data_offset": 0,
        "dimensions": [
            {"label": "column_index", "size": 2, "stride": 4},
            {"label": "row_index", "size": 2, "stride": 2},
        ],
        "encoding": MODULE.LAYER_ENCODING,
        "name": name,
        "value_count": 4,
    }


def make_fixture(case_name, stamp_ns, selected_index, exact_tf, elevation, accessibility):
    root_name = f"{case_name}.plane_seg_paired"
    elevation_path = f"{root_name}/frames/frame/layer_00_elevation.f32le"
    accessibility_path = f"{root_name}/frames/frame/layer_01_accessibility.f32le"
    points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
    quadrangles = {
        "header": stamp_header(stamp_ns),
        "points_xyz": points,
        "shape": {"height": 1, "point_count": 4, "width": 4},
    }
    if exact_tf:
        transform = exact_transform(stamp_ns)
        tf = {
            "exact_dynamic_transforms": [deepcopy(transform)],
            "required_transform_match_count": 1,
            "status": "exact_required_dynamic_tf",
            "world_to_base_link": transform,
        }
    else:
        tf = {
            "exact_dynamic_transforms": [],
            "required_transform_match_count": 0,
            "status": "missing_exact_required_dynamic_tf",
            "world_to_base_link": None,
        }
    frame = {
        "expected_quadrangles": quadrangles,
        "expected_quadrangles_sha256": sha256(canonical_json_bytes(quadrangles)),
        "grid_map": {
            "basic_layers": ["elevation"],
            "geometry": {
                "length_x": 1.0,
                "length_y": 1.0,
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "position_xyz": [4.0, 5.0, 6.0],
                "resolution": 0.5,
            },
            "header": stamp_header(stamp_ns),
            "inner_start_index": 0,
            "layers": [
                layer_metadata("elevation", elevation_path, elevation),
                layer_metadata("accessibility", accessibility_path, accessibility),
            ],
            "outer_start_index": 1,
        },
        "selected_frame_index": selected_index,
        "tf": tf,
    }
    fixture = {
        "frames": [frame],
        "hashing": {"algorithm": "sha256"},
        "schema": MODULE.PAIRED_FIXTURE_SCHEMA,
        "source": {
            "bag_path": f"bags/{case_name}.bag",
            "bag_sha256": sha256(f"source:{case_name}".encode("ascii")),
        },
    }
    blobs = {
        elevation_path: elevation,
        accessibility_path: accessibility,
    }
    factory_xyz = b"".join(struct.pack("<fff", *point) for point in points)
    return fixture, blobs, factory_xyz


def write_manifest(root, manifest):
    manifest_bytes = formatted_json_bytes(manifest)
    (root / "manifest.json").write_bytes(manifest_bytes)
    (root / "manifest.json.sha256").write_text(
        f"{sha256(manifest_bytes)}  manifest.json\n",
        encoding="ascii",
        newline="\n",
    )


def build_synthetic_corpus(root):
    nan_a = struct.pack("<I", 0x7FC01234)
    nan_b = struct.pack("<I", 0x7FC05678)
    cases = [
        (
            "z_exact",
            20_000_000_123,
            7,
            True,
            nan_a + struct.pack("<3f", 2.0, 3.0, 4.0),
            struct.pack("<2f", 0.1, 0.5) + nan_b + struct.pack("<f", 1.0),
        ),
        (
            "a_missing",
            10_000_000_456,
            3,
            False,
            struct.pack("<4f", 1.0, 2.0, 3.0, 4.0),
            struct.pack("<4f", 0.9, 0.91, -1.0, float("nan")),
        ),
    ]
    manifest_entries = []
    expected = {}
    for case_name, stamp_ns, selected_index, has_tf, elevation, accessibility in cases:
        fixture, blobs, factory_xyz = make_fixture(
            case_name,
            stamp_ns,
            selected_index,
            has_tf,
            elevation,
            accessibility,
        )
        for relative, raw in blobs.items():
            path = root.joinpath(*Path(relative).parts)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        fixture_name = f"{case_name}.plane_seg_paired.json"
        fixture_bytes = formatted_json_bytes(fixture)
        (root / fixture_name).write_bytes(fixture_bytes)
        manifest_entries.append(
            {
                "blobs": [
                    {"path": path, "sha256": sha256(raw), "size_bytes": len(raw)}
                    for path, raw in blobs.items()
                ],
                "fixture_path": fixture_name,
                "fixture_sha256": sha256(fixture_bytes),
                "fixture_size_bytes": len(fixture_bytes),
                "selected_frame_count": 1,
                "source_bag_path": fixture["source"]["bag_path"],
                "source_bag_sha256": fixture["source"]["bag_sha256"],
            }
        )
        expected[case_name] = {
            "accessibility": accessibility,
            "elevation": elevation,
            "factory": factory_xyz,
            "selected_index": selected_index,
            "stamp_ns": stamp_ns,
        }
    manifest = {
        "fixtures": manifest_entries,
        "hashing": {"algorithm": "sha256"},
        "schema": MODULE.PAIRED_MANIFEST_SCHEMA,
    }
    write_manifest(root, manifest)
    return expected


def rewrite_first_fixture(root, mutator):
    manifest = json.loads((root / "manifest.json").read_bytes())
    entry = manifest["fixtures"][0]
    fixture_path = root / entry["fixture_path"]
    fixture = json.loads(fixture_path.read_bytes())
    mutator(fixture)
    for frame in fixture["frames"]:
        frame["expected_quadrangles_sha256"] = sha256(
            canonical_json_bytes(frame["expected_quadrangles"])
        )
    fixture_bytes = formatted_json_bytes(fixture)
    fixture_path.write_bytes(fixture_bytes)
    entry["fixture_sha256"] = sha256(fixture_bytes)
    entry["fixture_size_bytes"] = len(fixture_bytes)
    write_manifest(root, manifest)


def parse_records(replay):
    header = MODULE.HEADER_STRUCT.unpack_from(replay)
    return header, [
        MODULE.FRAME_STRUCT.unpack_from(
            replay, MODULE.HEADER_BYTES + index * MODULE.FRAME_RECORD_BYTES
        )
        for index in range(header[5])
    ]


def decoded_case_name(record):
    return record[0].split(b"\0", 1)[0].decode("utf-8")


def blob_from_ref(replay, record, blob_index):
    offset = record[41 + blob_index * 2]
    size = record[42 + blob_index * 2]
    return replay[offset : offset + size]


class PlaneSegReplayCompilerTest(unittest.TestCase):
    def test_synthetic_pack_is_deterministic_and_preserves_contract_bytes(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            expected = build_synthetic_corpus(root)

            replay = MODULE.compile_replay(root)
            self.assertEqual(replay, MODULE.compile_replay(root))
            header, records = parse_records(replay)
            self.assertEqual(header[0], MODULE.MAGIC)
            self.assertEqual(header[1:7], (1, 0, 128, 3, 2, 384))
            self.assertEqual(header[7], 0)
            self.assertEqual(header[8], 128)
            self.assertEqual(header[9], 128 + 2 * 384)
            self.assertEqual(header[10], len(replay))
            self.assertEqual(
                header[11], hashlib.sha256((root / "manifest.json").read_bytes()).digest()
            )
            self.assertEqual(header[12], hashlib.sha256(replay[128:]).digest())
            self.assertTrue(math.isclose(header[13], 1.0e-5, rel_tol=0.0, abs_tol=1.0e-12))
            self.assertTrue(math.isclose(header[14], 1.0e-5, rel_tol=0.0, abs_tol=1.0e-12))

            self.assertEqual(
                [decoded_case_name(record) for record in records],
                ["a_missing", "z_exact"],
            )
            missing, exact = records
            self.assertEqual(missing[2], expected["a_missing"]["selected_index"])
            self.assertEqual(exact[2], expected["z_exact"]["selected_index"])
            self.assertEqual(
                missing[3],
                MODULE.EXPECT_MISSING_TF | MODULE.HAS_FACTORY_ORACLE | MODULE.RUN_CORE,
            )
            self.assertEqual(
                exact[3],
                MODULE.HAS_EXACT_TF | MODULE.HAS_FACTORY_ORACLE | MODULE.RUN_CORE,
            )
            self.assertTrue(all(record[3] & MODULE.RUN_CORE for record in records))
            self.assertFalse(any(record[3] & MODULE.HAS_CORE_ORACLE for record in records))
            self.assertEqual(missing[18:21], (0.0, 0.0, 0.0))
            self.assertEqual(missing[21:25], (0.0, 0.0, 0.0, 1.0))
            self.assertEqual(exact[18:21], (1.0, 2.0, 3.0))
            self.assertEqual(exact[25:31], (0.0, 0.0, 0.0, 1.0, 0.0, 0.0))
            self.assertEqual(
                exact[31:34],
                tuple(
                    struct.unpack("<f", struct.pack("<f", value))[0]
                    for value in (0.9, 0.01, 45.0)
                ),
            )
            self.assertEqual(exact[34:37], (0, 0, 0))
            self.assertEqual(missing[37], 2)
            self.assertEqual(exact[37], 1)
            self.assertEqual([record[38] for record in records], [MODULE.UINT32_MAX] * 2)
            self.assertEqual([record[39] for record in records], [4, 4])
            self.assertEqual([record[47:49] for record in records], [(0, 0), (0, 0)])

            for record in records:
                case = decoded_case_name(record)
                elevation = blob_from_ref(replay, record, 0)
                accessibility = blob_from_ref(replay, record, 1)
                factory = blob_from_ref(replay, record, 2)
                self.assertEqual(elevation, expected[case]["elevation"])
                self.assertEqual(accessibility, expected[case]["accessibility"])
                self.assertEqual(factory, expected[case]["factory"])
                self.assertEqual(
                    record[49], hashlib.sha256(elevation + accessibility + factory).digest()
                )
            self.assertEqual(
                blob_from_ref(replay, exact, 0)[:4], struct.pack("<I", 0x7FC01234)
            )
            self.assertEqual(
                blob_from_ref(replay, exact, 1)[8:12], struct.pack("<I", 0x7FC05678)
            )

            output_path = root / "out" / "synthetic.x30rpl"
            digest, size = MODULE.write_replay(root, output_path)
            self.assertEqual(output_path.read_bytes(), replay)
            self.assertEqual(size, len(replay))
            self.assertEqual(digest, sha256(replay))
            self.assertEqual(
                MODULE.default_sidecar_path(output_path).read_text(encoding="ascii"),
                f"{digest}  synthetic.x30rpl\n",
            )

    def test_synthetic_semantic_validation_rejects_invalid_inputs(self):
        mutations = [
            (
                "stamp",
                lambda fixture: fixture["frames"][0]["expected_quadrangles"]["header"].update(
                    stamp_header(99_000_000_001)
                ),
                "stamp does not match GridMap stamp",
            ),
            (
                "frame",
                lambda fixture: fixture["frames"][0]["grid_map"]["header"].update(
                    {"frame_id": "map"}
                ),
                "must be world",
            ),
            (
                "labels",
                lambda fixture: fixture["frames"][0]["grid_map"]["layers"][0][
                    "dimensions"
                ][0].update({"label": "x"}),
                "dimensions must be column_index and row_index",
            ),
            (
                "strides",
                lambda fixture: fixture["frames"][0]["grid_map"]["layers"][0][
                    "dimensions"
                ][1].update({"stride": 1}),
                "strides must be",
            ),
            (
                "data_offset",
                lambda fixture: fixture["frames"][0]["grid_map"]["layers"][0].update(
                    {"data_offset": 1}
                ),
                "data_offset must be zero",
            ),
            (
                "geometry",
                lambda fixture: fixture["frames"][0]["grid_map"]["geometry"].update(
                    {"length_x": 2.0}
                ),
                "length_x does not match",
            ),
            (
                "blob_size",
                lambda fixture: fixture["frames"][0]["grid_map"]["layers"][0].update(
                    {"blob_size_bytes": 12}
                ),
                "blob_size_bytes does not match",
            ),
        ]
        for name, mutation, message in mutations:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary_directory:
                root = Path(temporary_directory)
                build_synthetic_corpus(root)
                rewrite_first_fixture(root, mutation)
                with self.assertRaisesRegex(ValueError, message):
                    MODULE.compile_replay(root)

    def test_synthetic_hash_validation_rejects_corruption(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_synthetic_corpus(root)
            manifest = json.loads((root / "manifest.json").read_bytes())
            blob_path = root.joinpath(*Path(manifest["fixtures"][0]["blobs"][0]["path"]).parts)
            raw = bytearray(blob_path.read_bytes())
            raw[0] ^= 0x01
            blob_path.write_bytes(raw)
            with self.assertRaisesRegex(ValueError, "blob SHA-256 mismatch"):
                MODULE.compile_replay(root)

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_synthetic_corpus(root)
            (root / "manifest.json.sha256").write_text(
                f"{'0' * 64}  manifest.json\n", encoding="ascii"
            )
            with self.assertRaisesRegex(ValueError, "manifest SHA-256 mismatch"):
                MODULE.compile_replay(root)

    def test_committed_pack_integrity_and_rebuild_equality(self):
        replay = PACK_PATH.read_bytes()
        sidecar = MODULE.default_sidecar_path(PACK_PATH).read_text(encoding="ascii")
        declared_hash, declared_name = sidecar.split()
        self.assertEqual(declared_name, PACK_PATH.name)
        self.assertEqual(declared_hash, sha256(replay))
        self.assertEqual(replay, MODULE.compile_replay(PAIRED_FIXTURE_ROOT))

        header, records = parse_records(replay)
        self.assertEqual(header[5], 12)
        self.assertEqual(header[10], len(replay))
        self.assertEqual(header[11], hashlib.sha256((PAIRED_FIXTURE_ROOT / "manifest.json").read_bytes()).digest())
        self.assertEqual(header[12], hashlib.sha256(replay[128:]).digest())
        self.assertEqual(
            [decoded_case_name(record) for record in records],
            sorted(decoded_case_name(record) for record in records),
        )
        self.assertEqual(
            sum(bool(record[3] & MODULE.EXPECT_MISSING_TF) for record in records),
            1,
        )
        self.assertTrue(all(record[3] & MODULE.RUN_CORE for record in records))
        self.assertTrue(all(record[3] & MODULE.HAS_FACTORY_ORACLE for record in records))
        self.assertTrue(all(not (record[3] & MODULE.HAS_CORE_ORACLE) for record in records))
        self.assertTrue(all(record[38] == MODULE.UINT32_MAX for record in records))
        self.assertTrue(all(record[47:49] == (0, 0) for record in records))
        for record in records:
            elevation = blob_from_ref(replay, record, 0)
            accessibility = blob_from_ref(replay, record, 1)
            factory = blob_from_ref(replay, record, 2)
            self.assertEqual(
                record[49], hashlib.sha256(elevation + accessibility + factory).digest()
            )


if __name__ == "__main__":
    unittest.main()
