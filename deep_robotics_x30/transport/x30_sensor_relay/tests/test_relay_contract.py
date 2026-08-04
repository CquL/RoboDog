"""X30 被动传感器链的跨主机静态合同测试。

测试同时检查 105 和 106 两侧源码。只构建单侧无法发现 magic/version、
端口、Topic 或只读部署边界发生偏移。
"""

import re
import unittest
from pathlib import Path


RELAY_ROOT = Path(__file__).resolve().parents[1]
X30_ROOT = RELAY_ROOT.parents[1]
FORWARDER_ROOT = RELAY_ROOT / "x30_sensor_forwarder_105"
RECEIVER_ROOT = (
    X30_ROOT
    / "docker"
    / "x30_livox_ros2_transfer"
    / "ws"
    / "src"
    / "x30_sensor_receiver"
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class RelayContractTest(unittest.TestCase):
    def test_wire_constants_match(self) -> None:
        """写入端和读取端的全部帧判别字段必须一致。"""
        writer = read(
            FORWARDER_ROOT
            / "catkin_ws"
            / "src"
            / "x30_sensor_forwarder_ros1"
            / "include"
            / "x30_sensor_relay"
            / "wire_protocol.hpp"
        )
        reader = read(
            RECEIVER_ROOT
            / "include"
            / "x30_sensor_relay"
            / "wire_protocol_reader.hpp"
        )

        for pattern in (
            r"kProtocolVersion\s*=\s*1",
            r"kFrameHeaderSize\s*=\s*32",
            r"kPointCloud\s*=\s*1",
            r"kImu\s*=\s*2",
            r"kOdometry\s*=\s*3",
        ):
            self.assertRegex(writer, pattern)
            self.assertRegex(reader, pattern)
        self.assertIn("{{'X', '3', '0', 'R'}}", writer)
        self.assertIn("{{'X', '3', '0', 'R'}}", reader)

    def test_ports_and_topics_match(self) -> None:
        """105 的输入和端口必须映射到预期的 106 ROS2 输出。"""
        config = read(
            FORWARDER_ROOT
            / "catkin_ws"
            / "src"
            / "x30_sensor_forwarder_ros1"
            / "config"
            / "forwarder.yaml"
        )
        launch = read(RECEIVER_ROOT / "launch" / "passive_receiver.launch.py")

        for port in ("56110", "56111", "56112"):
            self.assertIn(port, config)
            self.assertIn(port, launch)

        expected = {
            "point_cloud_topic": "/lidar_points",
            "imu_topic": "/imu/data",
            "odometry_topic": "/leg_odom",
        }
        for key, topic in expected.items():
            self.assertRegex(config, rf"{key}:\s*\"{re.escape(topic)}\"")

        for topic in (
            "/x30/lidar_points",
            "/x30/body_imu",
            "/x30/leg_odom",
        ):
            self.assertIn(topic, launch)

    def test_105_node_is_subscriber_only(self) -> None:
        """防止 105 节点意外获得写入原厂 ROS1 图的能力。"""
        source = read(
            FORWARDER_ROOT
            / "catkin_ws"
            / "src"
            / "x30_sensor_forwarder_ros1"
            / "src"
            / "x30_sensor_forwarder_node.cpp"
        )
        self.assertIn(".subscribe<sensor_msgs::PointCloud2>", source)
        self.assertIn(".subscribe<sensor_msgs::Imu>", source)
        self.assertIn(".subscribe<nav_msgs::Odometry>", source)
        for forbidden in (
            ".advertise<",
            ".publish(",
            ".serviceClient<",
            ".advertiseService(",
            ".setParam(",
        ):
            self.assertNotIn(forbidden, source)

    def test_105_build_uses_bundled_catkin_toplevel(self) -> None:
        """迁移构建不得依赖失效的原厂软链接。"""
        build_script = read(
            FORWARDER_ROOT / "remote" / "00_build_105.sh"
        )
        toplevel = read(
            FORWARDER_ROOT / "remote" / "catkin_workspace_toplevel.cmake"
        )

        self.assertNotIn("catkin_init_workspace", build_script)
        self.assertIn("catkin_workspace_toplevel.cmake", build_script)
        self.assertIn('if [[ -L "${TOPLEVEL_FILE}" ]]', build_script)
        self.assertIn('rm -f -- "${TOPLEVEL_FILE}"', build_script)
        self.assertIn("--force-cmake", build_script)
        self.assertIn("CATKIN_TOPLEVEL_FIND_PACKAGE", toplevel)
        self.assertIn(
            'PATHS "/opt/ros/noetic/share/catkin/cmake"',
            toplevel,
        )
        self.assertIn("catkin_workspace()", toplevel)

    def test_passive_container_does_not_claim_devices(self) -> None:
        """106 可接收复制数据，但不得占用或停止传感器设备。"""
        script = read(
            X30_ROOT
            / "docker"
            / "x30_livox_ros2_transfer"
            / "remote"
            / "26_run_passive_relay.sh"
        )
        for forbidden in (
            "--privileged",
            "--device",
            "02_factory_livox_off.sh",
            "11_factory_imu_off.sh",
        ):
            self.assertNotIn(forbidden, script)
        self.assertIn("--network host", script)
        self.assertIn("x30_sensor_receiver", script)
        self.assertIn('readonly NAME="x30_ros2_passive"', script)

        stop_script = read(
            X30_ROOT
            / "docker"
            / "x30_livox_ros2_transfer"
            / "remote"
            / "29_stop_passive_relay.sh"
        )
        self.assertIn('readonly NAME="x30_ros2_passive"', stop_script)
        self.assertNotIn('docker rm -f "${NAME}" >/dev/null 2>&1 || true',
                         stop_script)


if __name__ == "__main__":
    unittest.main()
