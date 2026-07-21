import importlib.util
import math
import struct
import sys
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "analyze_x30_gridmap_baseline.py"
)
SPEC = importlib.util.spec_from_file_location("gridmap_analyzer", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class GridmapBaselineAnalyzerTest(unittest.TestCase):
    @staticmethod
    def pack_string(value):
        encoded = value.encode("utf-8")
        return struct.pack("<I", len(encoded)) + encoded

    def test_parse_grid_map(self):
        dimensions = (
            struct.pack("<I", 2)
            + self.pack_string("column_index")
            + struct.pack("<II", 2, 6)
            + self.pack_string("row_index")
            + struct.pack("<II", 3, 3)
        )
        payload = (
            struct.pack("<III", 7, 12, 34)
            + self.pack_string("world")
            + struct.pack("<ddd", 1.0, 2.0, 3.0)
            + struct.pack("<ddd", 10.0, 20.0, 0.0)
            + struct.pack("<dddd", 0.0, 0.0, 0.0, 1.0)
            + struct.pack("<I", 1)
            + self.pack_string("elevation")
            + struct.pack("<I", 1)
            + self.pack_string("elevation")
            + struct.pack("<I", 1)
            + dimensions
            + struct.pack("<I", 0)
            + struct.pack("<I", 6)
            + struct.pack("<6f", 0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
            + struct.pack("<HH", 1, 2)
        )

        frame = MODULE.parse_grid_map(payload, 99)
        self.assertEqual(frame.sequence, 7)
        self.assertEqual(frame.stamp_ns, 12_000_000_034)
        self.assertEqual(frame.frame_id, "world")
        self.assertEqual(frame.resolution, 1.0)
        self.assertEqual((frame.length_x, frame.length_y), (2.0, 3.0))
        self.assertEqual(frame.position, (10.0, 20.0, 0.0))
        self.assertEqual(frame.basic_layers, ("elevation",))
        self.assertEqual(frame.outer_start_index, 1)
        self.assertEqual(frame.inner_start_index, 2)
        self.assertEqual(frame.layer("elevation").values, tuple(range(6)))
        self.assertEqual(
            frame.layer("elevation").raw_data,
            struct.pack("<6f", 0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
        )

    def test_parse_grid_map_preserves_nan_payload_bits(self):
        raw_nan = struct.pack("<I", 0x7FC01234)
        dimensions = (
            struct.pack("<I", 2)
            + self.pack_string("column_index")
            + struct.pack("<II", 1, 1)
            + self.pack_string("row_index")
            + struct.pack("<II", 1, 1)
        )
        payload = (
            struct.pack("<III", 1, 2, 3)
            + self.pack_string("world")
            + struct.pack("<ddd", 1.0, 1.0, 1.0)
            + struct.pack("<ddd", 0.0, 0.0, 0.0)
            + struct.pack("<dddd", 0.0, 0.0, 0.0, 1.0)
            + struct.pack("<I", 1)
            + self.pack_string("elevation")
            + struct.pack("<I", 0)
            + struct.pack("<I", 1)
            + dimensions
            + struct.pack("<I", 7)
            + struct.pack("<I", 1)
            + raw_nan
            + struct.pack("<HH", 0, 0)
        )

        layer = MODULE.parse_grid_map(payload, 4).layer("elevation")
        self.assertTrue(math.isnan(layer.values[0]))
        self.assertEqual(layer.raw_data, raw_nan)
        self.assertEqual(layer.data_offset, 7)

    def test_parse_pose_stamped(self):
        frame_id = b"world"
        payload = (
            struct.pack("<III", 7, 12, 34)
            + struct.pack("<I", len(frame_id))
            + frame_id
            + struct.pack("<ddd", 1.0, 2.0, 3.0)
            + struct.pack("<dddd", 0.0, 0.0, 0.5, 0.5)
        )

        pose = MODULE.parse_pose_stamped(payload, 99)
        self.assertEqual(pose.stamp_ns, 12_000_000_034)
        self.assertEqual(pose.frame_id, "world")
        self.assertEqual(pose.position, (1.0, 2.0, 3.0))
        self.assertEqual(pose.orientation, (0.0, 0.0, 0.5, 0.5))

    def test_parse_tf_message_preserves_and_normalizes_frame_ids(self):
        def transform_payload(sequence, sec, nsec, frame_id, child_frame_id):
            return (
                struct.pack("<III", sequence, sec, nsec)
                + self.pack_string(frame_id)
                + self.pack_string(child_frame_id)
                + struct.pack("<ddd", 1.0, 2.0, 3.0)
                + struct.pack("<dddd", 0.0, 0.0, 0.0, 1.0)
            )

        payload = (
            struct.pack("<I", 2)
            + transform_payload(7, 12, 34, "/world", "/base_link")
            + transform_payload(8, 56, 78, "/map/local", "sensor/link")
        )
        message = MODULE.parse_tf_message(payload, 99, "/tf")

        self.assertEqual(message.record_time_ns, 99)
        self.assertEqual(message.topic, "/tf")
        self.assertEqual(len(message.transforms), 2)
        first = message.transforms[0]
        self.assertEqual(first.stamp_ns, 12_000_000_034)
        self.assertEqual(first.frame_id, "/world")
        self.assertEqual(first.child_frame_id, "/base_link")
        self.assertEqual(first.normalized_frame_id, "world")
        self.assertEqual(first.normalized_child_frame_id, "base_link")
        self.assertEqual(first.translation, (1.0, 2.0, 3.0))
        self.assertEqual(first.rotation, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(
            message.transforms[1].normalized_frame_id, "map/local"
        )

    def test_single_pass_inputs_keep_mode_topics_separate(self):
        mode_connection = MODULE.BagConnection(
            1, "/height_map_mode", "std_msgs/Int32"
        )
        state_connection = MODULE.BagConnection(
            2, "/height_map_mode_state", "std_msgs/Int32"
        )
        tf_connection = MODULE.BagConnection(3, "/tf", "tf2_msgs/TFMessage")
        messages = iter(
            [
                MODULE.BagMessage(mode_connection, 10, struct.pack("<i", 20)),
                MODULE.BagMessage(state_connection, 11, struct.pack("<i", 3)),
                MODULE.BagMessage(state_connection, 12, struct.pack("<i", 3)),
                MODULE.BagMessage(tf_connection, 13, struct.pack("<I", 0)),
            ]
        )
        bag_path = Path("synthetic.bag")

        with mock.patch.object(
            MODULE, "iter_rosbag_messages", return_value=messages
        ) as reader:
            inputs = MODULE.read_rosbag_plane_seg_inputs(bag_path)

        reader.assert_called_once_with(bag_path)
        self.assertEqual(inputs.height_map_mode_values, {20: 1})
        self.assertEqual(inputs.height_map_mode_state_values, {3: 2})
        self.assertEqual(len(inputs.tf_messages), 1)
        self.assertFalse(inputs.tf_static_messages)

    def test_decode_factory_tcp_frame(self):
        points = ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0), (10.0, 11.0, 12.0))
        point_data = b"".join(struct.pack("<fff", *point) for point in points)
        frame_len = 40 + len(point_data)
        frame = (
            struct.pack("<IIIII", 1, frame_len, 12345, 1, len(point_data))
            + point_data
            + struct.pack("<fIIIII", 0.1, 12, 34, 56, 78, 54321)
        )

        decoded = MODULE.decode_factory_tcp_stream(frame)
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].points, points)
        self.assertEqual(decoded[0].source_stamp_ns, 12_000_000_034)
        self.assertEqual(decoded[0].head_id, 12345)
        self.assertEqual(decoded[0].end_id, 54321)

    def test_quadrangle_metrics(self):
        square = ((0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 1.0))
        metrics = MODULE.quadrangle_metrics(square)
        self.assertAlmostEqual(metrics.area_m2, 1.0)
        self.assertAlmostEqual(metrics.z_range_m, 0.0)
        self.assertAlmostEqual(metrics.planarity_error_m, 0.0)
        self.assertFalse(metrics.degenerate)

        duplicate = (square[0], square[1], square[2], square[2])
        metrics = MODULE.quadrangle_metrics(duplicate)
        self.assertEqual(metrics.duplicate_vertices, 1)
        self.assertTrue(metrics.degenerate)

    def test_points_match(self):
        left = ((1.0, 2.0, 3.0),)
        right = ((1.0, 2.0, 3.0 + 1e-7),)
        matches, error = MODULE.points_match(left, right)
        self.assertTrue(matches)
        self.assertLess(error, 1e-6)

        mismatch, error = MODULE.points_match(left, ((1.0, 2.0, 3.1),))
        self.assertFalse(mismatch)
        self.assertGreater(error, 0.09)

    def test_reassemble_tcp_payload(self):
        data = MODULE.reassemble_tcp_payload(
            [(104, b"ef"), (100, b"abcd"), (102, b"cdef")]
        )
        self.assertEqual(data, b"abcdef")


if __name__ == "__main__":
    unittest.main()
