from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_mountain_gait_protocol_codes_match_x30_specification():
    protocol = read("include/deep_robotics/x30_udp_protocol.h")
    protocol_test = read("tests/x30_udp_protocol_test.cpp")

    assert "kGaitWalkSuffix = 0x00010300" in protocol
    assert "kGaitLWalkSuffix = 0x00010420" in protocol
    assert "kGaitMountainSuffix = 0x00010421" in protocol
    assert "kRemoteCommandPrefix = 0x21000000" in protocol
    assert "kNavigationCommandPrefix = 0x31000000" in protocol
    assert "navigation_mountain == 0x31010421u" in protocol_test
    assert "navigation_walk == 0x31010300u" in protocol_test
    assert "navigation_l_walk == 0x31010420u" in protocol_test
    assert "expected_mountain" in protocol_test


def test_x30_adapter_exposes_only_walk_and_mountain_gait_actions():
    constants_h = read("include/robot_hardware_constant.h")
    constants_cpp = read("src/robot_hardware_constant.cpp")
    adapter = read("src/deep_robotics/deep_robotics_x30.cpp")
    interface = read("include/robot_hardware_interface.h")

    assert "ACTION_GAIT_WALK" in constants_h
    assert "ACTION_GAIT_L_WALK" in constants_h
    assert "ACTION_GAIT_MOUNTAIN" in constants_h
    assert 'ACTION_GAIT_WALK = "gait_walk"' in constants_cpp
    assert 'ACTION_GAIT_L_WALK = "gait_l_walk"' in constants_cpp
    assert 'ACTION_GAIT_MOUNTAIN = "gait_mountain"' in constants_cpp
    assert "sendGaitCommand" in adapter
    assert "kGaitMountainSuffix" in adapter
    assert "kGaitWalkSuffix" in adapter
    assert "kGaitLWalkSuffix" in adapter
    assert "Unsupported action" in adapter
    assert "ERROR_ROBOT_HARDWARE_ACTION_FAILED" in adapter
    assert "ACTION_GAIT_STAIRS_45" not in constants_h

    assert "virtual int32_t initRobotHardware() = 0;" in interface
    assert "virtual int32_t writeRobotVelocityCommand" in interface
    assert "virtual int32_t writeActionCommand" in interface


def test_gait_test_verifies_ros1_state_and_restores_walk():
    source = read("robot_test_x30_gait.cpp")
    cmake = read("CMakeLists.txt")

    assert "BUILD_X30_ROS1_GAIT_TEST" in cmake
    assert "robot_test_x30_gait" in cmake
    assert '"/robot_basic_state"' in source
    assert '"/robot_gait_state"' in source
    assert '"/control_mode"' in source
    assert "std_msgs::UInt8" in source
    assert "kMountainGaitState = 33" in source
    assert "kWalkGaitState = 0" in source
    assert "kLWalkGaitState = 32" in source
    assert "ACTION_GAIT_MOUNTAIN" in source
    assert "ACTION_GAIT_WALK" in source
    assert "ACTION_GAIT_L_WALK" in source
    assert "ACTION_START_STOP_MOTION" not in source
    assert "RobotVelocityCommand zero{0.0, 0.0, 0.0}" in source
    assert "Mountain test finished; L-walk gait restored" in source
    assert "configure_non_manual_mode" in source
    assert "configure_navigation_velocity_source" in source


def test_mountain_move_is_explicit_bounded_and_state_guarded():
    source = read("robot_test_x30_gait.cpp")

    assert 'arg == "mountain_move"' in source
    assert '"CONFIRM_MOUNTAIN_MOVE"' in source
    assert "kMountainMoveMaxVxMps = 0.5" in source
    assert "kMountainMoveDefaultMaxDurationS = 30.0" in source
    assert "kSensorFeedbackFreshnessMs = 500" in source
    assert "kMotionStateFreshnessMs = 2500" in source
    assert "kControlModeFreshnessMs = 2500" in source
    assert 'config["mountain_move_max_duration_s"]' in source
    assert "runMountainForward" in source
    assert "isSupportedMountainStartGait" not in source
    assert "start gait must be walk (0) or L_walk (32)" not in source
    assert "Requesting mountain directly; no intermediate walk gait is required" in source
    assert "stateIsFresh" in source
    assert "kNonManualControlMode = 1" in source
    assert "control_mode.value != kNonManualControlMode" in source
    assert "gait.value != kMountainGaitState" in source
    assert "!isMotionCapableBasicState(basic.value)" in source
    assert "RobotVelocityCommand command{forward_mps, 0.0, 0.0}" in source
    assert "robot->writeRobotVelocityCommand(command)" in source
    assert "bestEffortStop(robot)" in source
    assert '"/leg_odom"' in source
    assert '"/robot_velocity"' in source
    assert "command_distance_m" in source
    assert "avg_odom_speed_mps" in source
    assert "avg_feedback_velocity_mps" in source
    assert "kMinimumMotionDistanceM = 0.005" in source
    assert "kMinimumFeedbackSpeedMps = 0.01" in source
    assert "motion_detected" in source
    assert "movement_success" in source
    assert "No motion detected by /leg_odom" in source
    assert "Distance after zero/stop hold" in source
    assert 'backend != "udp_navigation"' in source
    assert 'command_source != "navigation"' in source
    assert "ACTION_START_STOP_MOTION" not in source
