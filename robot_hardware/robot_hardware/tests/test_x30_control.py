from pathlib import Path


# 源码契约测试补充编译后的数据包测试，用于固定预期 CLI、构建选项、
# 适配器连接方式和只读诊断边界。
ROOT = Path(__file__).resolve().parents[1]


# 文件访问相对源码树定位，使测试可从任意构建目录或 CI 工作目录运行。
def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


# 已退役的专用现场工具不得与统一 CLI 同时重新出现。
def test_unified_x30_cli_is_the_only_field_test_target():
    source = read("robot_test_x30.cpp")
    cmake = read("CMakeLists.txt")

    for command in ['"status"', '"zero"', '"gait"', '"move"']:
        assert command in source
    for gait in ['"walk"', '"l_walk"', '"mountain"', '"stairs"']:
        assert gait in source

    assert '"CONFIRM_MOVE"' in source
    assert "RobotFactory::RobotAllocate" in source
    assert "ACTION_GAIT_WALK" in source
    assert "ACTION_GAIT_L_WALK" in source
    assert "ACTION_GAIT_MOUNTAIN" in source
    assert "ACTION_GAIT_STAIRS" in source
    assert "No intermediate or recovery gait is sent" in source
    assert "ACTION_START_STOP_MOTION" not in source
    assert "mountain_move_max_duration_s" not in source
    assert "stairs_move_max_duration_s" not in source

    assert "add_executable(robot_test_x30 robot_test_x30.cpp)" in cmake
    assert "BUILD_X30_ROS1_TEST" in cmake
    assert "X30_TEST_WITH_ROS1" in cmake
    assert "robot_test_x30_udp" not in cmake
    assert "robot_test_x30_udp_odom" not in cmake
    assert "robot_test_x30_gait" not in cmake
    assert "robot_test_x30_stair" not in cmake

    for retired_source in [
        "robot_test_x30_udp.cpp",
        "robot_test_x30_udp_odom.cpp",
        "robot_test_x30_gait.cpp",
        "robot_test_x30_stair.cpp",
    ]:
        assert not (ROOT / retired_source).exists()


# 运动命令接收全部机身坐标分量并遵循调用方提供的时长；
# 仅受配置包络限制，不含隐藏固定时长策略。
def test_unified_move_accepts_velocity_turn_and_requested_duration():
    source = read("robot_test_x30.cpp")
    config = read("config.yaml")

    assert "RobotVelocityCommand command{vx, vy, omega}" in source
    assert "std::chrono::duration<double>(duration_s)" in source
    assert "robot->writeRobotVelocityCommand(command)" in source
    assert "constexpr int kCommandHz = 50;" in source
    assert "sendZeroFor(robot, kZeroHoldMs)" in source
    assert "commands_sent=" in source
    assert "elapsed_s=" in source
    assert 'config["max_vx"]' in source
    assert 'config["max_vy"]' in source
    assert 'config["max_omega"]' in source
    assert "max_vx: 1.0" in config
    assert "max_vy: 0.5" in config
    assert "max_omega: 1.2" in config
    assert "max_duration" not in config


# ROS1 支持可增加观测与验证，但不得替代纯数据包构建使用的
# 通用 HAL 命令路径。
def test_ros1_build_adds_feedback_without_changing_control_commands():
    source = read("robot_test_x30.cpp")

    assert "#ifdef X30_TEST_WITH_ROS1" in source
    assert '"/robot_basic_state"' in source
    assert '"/robot_gait_state"' in source
    assert '"/control_mode"' in source
    assert '"/height_map_mode_state"' in source
    assert '"/leg_odom"' in source
    assert '"/robot_velocity"' in source
    assert "waitForGait" in source
    assert "planarDistance" in source
    assert "status requires BUILD_X30_ROS1_TEST=ON" in source


# 将步态后缀固定为已恢复的厂家命令值及其组合后的 navigation 来源命令码。
def test_x30_gait_protocol_codes_match_the_factory_specification():
    protocol = read("include/deep_robotics/x30_udp_protocol.h")
    protocol_test = read("tests/x30_udp_protocol_test.cpp")

    expected = {
        "kGaitWalkSuffix = 0x00010300": "navigation_walk == 0x31010300u",
        "kGaitLWalkSuffix = 0x00010420": "navigation_l_walk == 0x31010420u",
        "kGaitMountainSuffix = 0x00010421": (
            "navigation_mountain == 0x31010421u"
        ),
        "kGaitStairsSuffix = 0x00010405": (
            "navigation_stairs == 0x31010405u"
        ),
    }
    for protocol_text, test_text in expected.items():
        assert protocol_text in protocol
        assert test_text in protocol_test


# X30 适配器必须保持在三函数通用接口之后，
# 并在具体实现中应用文档规定的普通楼梯限幅。
def test_x30_adapter_maps_the_common_interface_and_stair_limits():
    constants_h = read("include/robot_hardware_constant.h")
    adapter_h = read("include/deep_robotics/deep_robotics_x30.h")
    adapter_cpp = read("src/deep_robotics/deep_robotics_x30.cpp")
    interface = read("include/robot_hardware_interface.h")

    for action in [
        "ACTION_GAIT_WALK",
        "ACTION_GAIT_L_WALK",
        "ACTION_GAIT_MOUNTAIN",
        "ACTION_GAIT_STAIRS",
    ]:
        assert action in constants_h
        assert action in adapter_cpp

    assert "virtual int32_t initRobotHardware() = 0;" in interface
    assert "virtual int32_t writeRobotVelocityCommand" in interface
    assert "virtual int32_t writeActionCommand" in interface
    assert "ordinary_stairs_limits_active_" in adapter_h
    assert "x30_udp_protocol::kGaitStairsMaxVx" in adapter_cpp
    assert "x30_udp_protocol::kGaitStairsMaxVy" in adapter_cpp
    assert "x30_udp_protocol::kGaitStairsMaxOmega" in adapter_cpp


# 验证三条速度路径均保持实现：Axis 模拟、直接 physical 毫单位，
# 以及带 watchdog 的 navigation float64 twist 数据包。
def test_x30_velocity_backends_match_the_recovered_protocol():
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

    assert "UDP_PHYSICAL" in adapter
    assert "UDP_NAVIGATION" in adapter
    assert "sendPhysicalVelocityCommand" in adapter
    assert "sendNavigationVelocityCommand" in adapter
    assert "velocityWatchdogLoop" in adapter

    assert 'velocity_backend: "udp_navigation"' in config
    assert "navigation_velocity_port: 43897" in config
    assert "configure_navigation_velocity_source: false" in config
    assert "expected_navigation" in protocol_test


# 诊断采集器可以检查 ROS 和网络流量，但自身绝不能发布命令或发送数据包。
def test_x30_diagnostic_capture_scripts_remain_read_only():
    collect = read("scripts/x30_collect_control_stack.sh")
    capture = read("scripts/x30_capture_motion_udp.sh")
    analyzer = read("scripts/analyze_x30_motion_pcap.py")

    assert "rosnode info" in collect
    assert "rostopic pub" not in collect
    assert "sendto" not in collect

    assert "tcpdump" in capture
    assert "192.168.1.103" in capture
    assert "43893" in capture
    assert "rostopic pub" not in capture

    assert "read_udp_packets" in analyzer
    assert "import scapy" not in analyzer
    assert "import dpkt" not in analyzer
