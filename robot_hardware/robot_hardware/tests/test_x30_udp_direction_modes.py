from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_x30_udp_test_exposes_safe_direction_modes():
    source = read("robot_test_x30_udp.cpp")

    for mode in [
        '"zero"',
        '"list"',
        '"forward"',
        '"backward"',
        '"left"',
        '"right"',
        '"yaw_left"',
        '"yaw_right"',
        '"all_safe"',
        '"forward_calib"',
        '"backward_calib"',
        '"left_calib"',
        '"right_calib"',
        '"all_calib"',
        '"forward_1m"',
        '"backward_1m"',
        '"left_1m"',
        '"right_1m"',
        '"all_1m"',
    ]:
        assert mode in source

    assert "kSafeTestVx" in source
    assert "kSafeTestVy" in source
    assert "kSafeTestOmega" in source
    assert "kOneMeterDistance" in source
    assert "kDistanceTestVx" in source
    assert "kDistanceTestVy" in source
    assert "kCalibrationTestVx" in source
    assert "kCalibrationTestVy" in source
    assert "kCalibrationTestDurationMs" in source
    assert "durationFromDistance" in source
    assert "distanceTests" in source
    assert "calibrationTests" in source
    assert "runDirectionTest" in source
    assert "backendUsesMotionToggle" in source
    assert "printUsage" in source


def test_x30_udp_test_documents_protocol_boundaries():
    source = read("robot_test_x30_udp.cpp")

    assert "0x21010130" in source
    assert "0x21010131" in source
    assert "0x21010135" in source
    assert "0x21010201" in source
    assert "ROS /cmd_vel" in source
    assert "constexpr int kCommandHz = 50;" in source


def test_x30_odom_distance_test_is_separate_ros1_target():
    cmake = read("CMakeLists.txt")
    odom_source = read("robot_test_x30_udp_odom.cpp")

    assert "BUILD_X30_ROS1_ODOM_TEST" in cmake
    assert "find_package(catkin" in cmake
    assert "roscpp" in cmake
    assert "nav_msgs" in cmake
    assert "std_msgs" in cmake
    assert "robot_test_x30_udp_odom" in cmake

    assert '"/leg_odom"' in odom_source
    assert "nav_msgs/Odometry.h" in odom_source
    assert "writeRobotVelocityCommand" in odom_source
    assert "ACTION_STOP_MOVE" in odom_source
    assert "target_distance_m" in odom_source
    assert "max_duration_s" in odom_source
    assert "std::hypot" in odom_source
    assert '"/robot_velocity"' in odom_source
    assert "geometry_msgs/Twist.h" in odom_source
    assert "std_msgs/Int32.h" in odom_source
    assert '"/robot_basic_state"' in odom_source
    assert "BasicStateTracker" in odom_source
    assert '"before motion toggle"' in odom_source
    assert 'printBasicState("after motion toggle"' in odom_source
    assert "does not match documented stepping state 4" in odom_source
    assert "avg_odom_speed_mps" in odom_source
    assert "speed_scale" in odom_source
    assert "feedback_velocity_mps" in odom_source
    assert "constexpr int kCommandHz = 50;" in odom_source
    assert "backendUsesMotionToggle" in odom_source


def test_x30_diagnostic_scripts_are_read_only_and_capture_factory_udp():
    collect = read("scripts/x30_collect_control_stack.sh")
    capture = read("scripts/x30_capture_motion_udp.sh")

    assert "rosnode info" in collect
    assert "/udp_sender" in collect
    assert "rosparam get /udp_sender" in collect
    assert "message_transformer_106.launch" in collect
    assert "43893" in collect
    assert "network.toml" in collect
    assert "rostopic pub" not in collect
    assert "sendto" not in collect

    assert "tcpdump" in capture
    assert "192.168.1.103" in capture
    assert "43893" in capture
    assert "rostopic pub" not in capture
    assert "robot_test_x30" not in capture


def test_x30_pcap_analyzer_has_no_external_dependencies():
    analyzer = read("scripts/analyze_x30_motion_pcap.py")

    assert "read_udp_packets" in analyzer
    assert "Little-endian command codes" in analyzer
    assert "word1_as_float" in analyzer
    assert "import scapy" not in analyzer
    assert "import dpkt" not in analyzer


def test_x30_factory_velocity_backends_match_recovered_protocol():
    header = read("include/deep_robotics/x30_udp_protocol.h")
    adapter = read("src/deep_robotics/deep_robotics_x30.cpp")
    config = read("config.yaml")
    protocol_test = read("tests/x30_udp_protocol_test.cpp")

    assert "kPhysicalVxCode = 0x00000123" in header
    assert "kPhysicalVyCode = 0x00000124" in header
    assert "kPhysicalYawCode = 0x00000122" in header
    assert "kNavigationVelocityCode = 0x00000150" in header
    assert "velocity * 1000.0" in header
    assert "kNavigationVelocityDataSize = 24" in header
    assert "writeDoubleLittleEndian" in header

    assert "UDP_PHYSICAL" in adapter
    assert "UDP_NAVIGATION" in adapter
    assert "sendPhysicalVelocityCommand" in adapter
    assert "sendNavigationVelocityCommand" in adapter
    assert "velocityWatchdogLoop" in adapter
    assert "velocity_command_timeout_ms" in adapter

    assert 'velocity_backend: "udp_navigation"' in config
    assert "navigation_velocity_port: 43897" in config
    assert "configure_navigation_velocity_source: false" in config

    assert "expected_positive" in protocol_test
    assert "expected_negative" in protocol_test
    assert "expected_navigation" in protocol_test
