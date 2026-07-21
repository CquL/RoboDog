#include "unitree/unitree_h2.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Recorder = unitree::robot::test::H2LocoRecorder;

void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(float lhs, float rhs)
{
    return std::abs(lhs - rhs) < 1e-6f;
}

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
}

YAML::Node makeConfig(bool allow_motion = false, bool allow_actions = false)
{
    YAML::Node config;
    config["robot_model"] = "unitree_h2";
    config["network_interface_card_name"] = "fake-h2-nic";
    config["dds_domain_id"] = 7;
    config["sdk_timeout_s"] = 1.25f;
    config["velocity_command_duration_s"] = 0.30f;
    config["max_vx"] = 0.20;
    config["max_vy"] = 0.10;
    config["max_omega"] = 0.30;
    config["velocity_watchdog_hz"] = 200;
    config["velocity_command_timeout_ms"] = 30;
    config["velocity_zero_hold_ms"] = 100;
    config["required_motion_fsm_id"] = 601;
    config["verify_fsm_on_init"] = true;
    config["allow_motion_commands"] = allow_motion;
    config["allow_state_changing_actions"] = allow_actions;
    return config;
}

void testInitializationAndSafetyGates()
{
    Recorder::Reset();
    UnitreeH2 robot(makeConfig());
    require(robot.initRobotHardware() == CMD_SUCCESS, "H2 init failed");
    require(Recorder::channel_init_calls == 1, "ChannelFactory Init not called once");
    require(Recorder::domain_id == 7, "DDS domain was not forwarded");
    require(Recorder::interface_name == "fake-h2-nic", "DDS NIC was not forwarded");
    require(Recorder::loco_init_calls == 1, "LocoClient Init not called once");
    require(Recorder::timeout_calls == 1 && near(Recorder::last_timeout_s, 1.25f),
            "SDK timeout was not forwarded");
    require(Recorder::get_fsm_calls == 1, "GetFsmId readiness check missing");
    require(robot.initRobotHardware() == CMD_SUCCESS,
            "second initialization was not idempotent");
    require(Recorder::channel_init_calls == 1 && Recorder::loco_init_calls == 1,
            "second initialization repeated SDK setup");

    RobotVelocityCommand nonzero{0.01, 0.0, 0.0};
    require(robot.writeRobotVelocityCommand(nonzero) ==
                ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK,
            "default safety gate accepted non-zero velocity");
    require(Recorder::set_velocity_calls == 0,
            "rejected non-zero velocity reached SDK");

    RobotVelocityCommand tiny_nonzero{1e-12, 0.0, 0.0};
    require(robot.writeRobotVelocityCommand(tiny_nonzero) ==
                ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK,
            "tiny non-zero velocity bypassed the motion interlock");
    require(Recorder::set_velocity_calls == 0,
            "tiny rejected velocity reached SDK");

    RobotVelocityCommand invalid{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    require(robot.writeRobotVelocityCommand(invalid) == ERROR_ROBOT_HARDWARE_MOVE,
            "NaN velocity was not rejected");

    require(robot.writeActionCommand(ACTION_STAND_UP) ==
                ERROR_ROBOT_HARDWARE_SAFETY_INTERLOCK,
            "default action gate accepted StandUp");
    require(Recorder::stand_up_calls == 0 && Recorder::stop_move_calls == 0,
            "rejected action reached SDK");

    RobotVelocityCommand zero{0.0, 0.0, 0.0};
    require(robot.writeRobotVelocityCommand(zero) == CMD_SUCCESS,
            "zero velocity did not reach SDK");
    require(Recorder::set_velocity_calls == 1 && near(Recorder::last_vx, 0.0f) &&
                near(Recorder::last_vy, 0.0f) && near(Recorder::last_omega, 0.0f),
            "zero velocity SDK mapping is wrong");

    require(robot.writeActionCommand(ACTION_STOP_MOVE) == CMD_SUCCESS,
            "stop_move did not map to StopMove");
    require(Recorder::stop_move_calls == 1, "StopMove SDK call missing");
}

void testVelocityAndActionMapping()
{
    Recorder::Reset();
    UnitreeH2 robot(makeConfig(true, true));
    require(robot.initRobotHardware() == CMD_SUCCESS, "enabled H2 init failed");

    RobotVelocityCommand command{2.0, -2.0, 3.0};
    require(robot.writeRobotVelocityCommand(command) == CMD_SUCCESS,
            "enabled velocity command failed");
    require(Recorder::set_velocity_calls == 1, "SetVelocity call missing");
    require(near(Recorder::last_vx, 0.20f) && near(Recorder::last_vy, -0.10f) &&
                near(Recorder::last_omega, 0.30f) &&
                near(Recorder::last_duration_s, 0.30f),
            "velocity clamp/duration mapping is wrong");

    require(robot.writeActionCommand(ACTION_STAND_UP) == CMD_SUCCESS,
            "stand_up mapping failed");
    require(robot.writeActionCommand(ACTION_PREPARE_MOTION) == CMD_SUCCESS,
            "prepare_motion mapping failed");
    require(robot.writeActionCommand(ACTION_DAMP) == CMD_SUCCESS,
            "damp mapping failed");
    require(robot.writeActionCommand(ACTION_SQUAT) == CMD_SUCCESS,
            "squat mapping failed");
    require(robot.writeActionCommand(ACTION_SIT) == CMD_SUCCESS,
            "sit mapping failed");
    require(Recorder::stand_up_calls == 1 && Recorder::start_calls == 1 &&
                Recorder::damp_calls == 1 && Recorder::squat_calls == 1 &&
                Recorder::sit_calls == 1,
            "one or more action strings mapped to the wrong SDK call");
    require(Recorder::stop_move_calls == 5,
            "state-changing actions were not preceded by StopMove");

    require(robot.writeActionCommand(ACTION_LIE_DOWN) ==
                ERROR_ROBOT_HARDWARE_NOT_SUPPORTED,
            "lie_down must remain unsupported for H2");
    require(Recorder::damp_calls == 1,
            "lie_down was incorrectly mapped to Damp");
    require(Recorder::stop_move_calls == 5,
            "unsupported lie_down unexpectedly reached the SDK");
    require(robot.writeActionCommand("unknown_h2_action") ==
                ERROR_ROBOT_HARDWARE_NOT_SUPPORTED,
            "unknown action was not rejected");
    require(Recorder::stop_move_calls == 5,
            "unknown action unexpectedly reached the SDK");
}

void testSdkFailuresAndWatchdog()
{
    Recorder::Reset();
    UnitreeH2 robot(makeConfig(true, true));
    require(robot.initRobotHardware() == CMD_SUCCESS, "failure-path H2 init failed");

    RobotVelocityCommand command{0.05, 0.0, 0.0};
    Recorder::set_velocity_result = -77;
    const int stops_before_error = Recorder::stop_move_attempts;
    require(robot.writeRobotVelocityCommand(command) == ERROR_ROBOT_HARDWARE_MOVE,
            "vendor velocity error was not normalized");
    require(Recorder::stop_move_attempts > stops_before_error,
            "ambiguous vendor velocity error did not trigger StopMove");

    Recorder::set_velocity_result = 0;
    Recorder::throw_on_set_velocity = true;
    const int stops_before_exception = Recorder::stop_move_attempts;
    require(robot.writeRobotVelocityCommand(command) == ERROR_ROBOT_HARDWARE_MOVE,
            "SDK velocity exception escaped the int32 contract");
    require(Recorder::stop_move_attempts > stops_before_exception,
            "post-delivery velocity exception did not trigger StopMove");
    Recorder::throw_on_set_velocity = false;

    Recorder::stop_move_result = -88;
    require(robot.writeActionCommand(ACTION_STOP_MOVE) ==
                ERROR_ROBOT_HARDWARE_STOP_MOVE,
            "vendor StopMove error was not normalized");
    Recorder::stop_move_result = 0;

    Recorder::throw_on_stop_move = true;
    require(robot.writeActionCommand(ACTION_STOP_MOVE) ==
                ERROR_ROBOT_HARDWARE_STOP_MOVE,
            "SDK StopMove exception escaped the int32 contract");
    Recorder::throw_on_stop_move = false;

    Recorder::stand_up_result = -66;
    require(robot.writeActionCommand(ACTION_STAND_UP) ==
                ERROR_ROBOT_HARDWARE_ACTION_FAILED,
            "vendor action error was not normalized");
    Recorder::stand_up_result = 0;

    Recorder::throw_on_action = true;
    require(robot.writeActionCommand(ACTION_STAND_UP) ==
                ERROR_ROBOT_HARDWARE_ACTION_FAILED,
            "SDK action exception escaped the int32 contract");
    Recorder::throw_on_action = false;

    const int attempts_before_watchdog = Recorder::stop_move_attempts;
    const int stops_before_watchdog = Recorder::stop_move_calls;
    Recorder::throw_on_stop_move = true;
    require(robot.writeRobotVelocityCommand(command) == CMD_SUCCESS,
            "watchdog setup velocity failed");
    require(waitUntil(
                [&]() { return Recorder::stop_move_attempts > attempts_before_watchdog; },
                std::chrono::milliseconds(500)),
            "velocity watchdog did not attempt StopMove after timeout");
    Recorder::throw_on_stop_move = false;
    require(waitUntil(
                [&]() { return Recorder::stop_move_calls > stops_before_watchdog; },
                std::chrono::milliseconds(500)),
            "velocity watchdog did not retry StopMove after an SDK exception");
}

void testReadinessAndInitializationFailures()
{
    Recorder::Reset();
    Recorder::get_fsm_result = -99;
    UnitreeH2 robot(makeConfig());
    require(robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY,
            "GetFsmId failure did not reject initialization");

    Recorder::Reset();
    Recorder::throw_on_get_fsm = true;
    UnitreeH2 throwing_readiness_robot(makeConfig());
    require(throwing_readiness_robot.initRobotHardware() ==
                ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY,
            "GetFsmId exception did not reject initialization");

    Recorder::Reset();
    Recorder::throw_on_channel_init = true;
    UnitreeH2 channel_failure_robot(makeConfig());
    require(channel_failure_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "ChannelFactory exception escaped initialization");

    Recorder::Reset();
    Recorder::throw_on_loco_init = true;
    UnitreeH2 loco_failure_robot(makeConfig());
    require(loco_failure_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "LocoClient exception escaped initialization");

    Recorder::Reset();
    YAML::Node no_readiness_config = makeConfig();
    no_readiness_config["verify_fsm_on_init"] = false;
    UnitreeH2 read_robot(no_readiness_config);
    require(read_robot.initRobotHardware() == CMD_SUCCESS,
            "H2 init without readiness probe failed");
    require(Recorder::get_fsm_calls == 0,
            "disabled readiness probe still called GetFsmId");
    int fsm_id = -1;
    require(read_robot.readFsmId(fsm_id) == CMD_SUCCESS && fsm_id == 601,
            "readFsmId success path is wrong");
    int fsm_mode = -1;
    require(read_robot.readFsmMode(fsm_mode) == CMD_SUCCESS && fsm_mode == 2,
            "readFsmMode success path is wrong");
    std::vector<int> ids;
    std::vector<std::string> names;
    require(read_robot.readAvailableFsmIds(ids, names) == CMD_SUCCESS &&
                ids.size() == 5 && ids.back() == 601 &&
                names.size() == ids.size(),
            "readAvailableFsmIds success path is wrong");
    Recorder::get_fsm_result = -55;
    require(read_robot.readFsmId(fsm_id) == ERROR_ROBOT_HARDWARE_STATE_STALE,
            "readFsmId vendor error was not normalized");
    Recorder::get_fsm_result = 0;
    Recorder::throw_on_get_fsm = true;
    require(read_robot.readFsmId(fsm_id) == ERROR_ROBOT_HARDWARE_STATE_STALE,
            "readFsmId exception escaped the int32 contract");
    Recorder::throw_on_get_fsm = false;
    Recorder::get_fsm_mode_result = -56;
    require(read_robot.readFsmMode(fsm_mode) == ERROR_ROBOT_HARDWARE_STATE_STALE,
            "readFsmMode vendor error was not normalized");
    Recorder::get_fsm_mode_result = 0;
    Recorder::throw_on_get_available_fsm = true;
    require(read_robot.readAvailableFsmIds(ids, names) ==
                ERROR_ROBOT_HARDWARE_STATE_STALE,
            "readAvailableFsmIds exception escaped the int32 contract");
}

void testInvalidConfigurationAndDestructorBoundary()
{
    Recorder::Reset();
    YAML::Node invalid_config = makeConfig();
    invalid_config["max_vx"] = std::numeric_limits<double>::quiet_NaN();
    UnitreeH2 invalid_robot(invalid_config);
    require(invalid_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "non-finite configuration was accepted");
    require(Recorder::channel_init_calls == 0,
            "invalid configuration reached ChannelFactory");

    Recorder::Reset();
    {
        UnitreeH2 robot(makeConfig());
        require(robot.initRobotHardware() == CMD_SUCCESS,
                "destructor-boundary H2 init failed");
        RobotVelocityCommand zero{0.0, 0.0, 0.0};
        require(robot.writeRobotVelocityCommand(zero) == CMD_SUCCESS,
                "destructor-boundary setup failed");
        Recorder::throw_on_stop_move = true;
    }
    Recorder::throw_on_stop_move = false;
}

void testLiveMotionHardSafetyCeilings()
{
    Recorder::Reset();
    YAML::Node excessive_velocity = makeConfig(true, false);
    excessive_velocity["max_vx"] = 0.21;
    UnitreeH2 velocity_robot(excessive_velocity);
    require(velocity_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "configuration above the absolute vx ceiling was accepted");
    require(Recorder::channel_init_calls == 0,
            "excessive vx configuration reached ChannelFactory");

    Recorder::Reset();
    YAML::Node excessive_duration = makeConfig(true, false);
    excessive_duration["velocity_command_duration_s"] = 0.31f;
    UnitreeH2 duration_robot(excessive_duration);
    require(duration_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "configuration above the absolute duration ceiling was accepted");

    Recorder::Reset();
    YAML::Node no_fsm_gate = makeConfig(true, false);
    no_fsm_gate["verify_fsm_on_init"] = false;
    UnitreeH2 no_fsm_robot(no_fsm_gate);
    require(no_fsm_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "motion was enabled while the FSM readiness gate was disabled");
    require(Recorder::channel_init_calls == 0,
            "missing FSM gate reached ChannelFactory");

    Recorder::Reset();
    Recorder::fsm_id = 1;
    UnitreeH2 wrong_initial_fsm_robot(makeConfig(true, false));
    require(wrong_initial_fsm_robot.initRobotHardware() ==
                ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY,
            "motion-enabled init accepted the wrong FSM ID");

    Recorder::Reset();
    UnitreeH2 changed_fsm_robot(makeConfig(true, false));
    require(changed_fsm_robot.initRobotHardware() == CMD_SUCCESS,
            "changed-FSM setup failed");
    Recorder::fsm_id = 1;
    RobotVelocityCommand nonzero{0.05, 0.0, 0.0};
    const int stops_before_rejection = Recorder::stop_move_attempts;
    require(changed_fsm_robot.writeRobotVelocityCommand(nonzero) ==
                ERROR_ROBOT_HARDWARE_CONTROL_NOT_READY,
            "per-command FSM change was not rejected");
    require(Recorder::set_velocity_calls == 0,
            "wrong-FSM non-zero velocity reached SetVelocity");
    require(Recorder::stop_move_attempts > stops_before_rejection,
            "wrong-FSM motion attempt did not request StopMove");
}

} // namespace

int main()
{
    try {
        testInitializationAndSafetyGates();
        testVelocityAndActionMapping();
        testSdkFailuresAndWatchdog();
        testReadinessAndInitializationFailures();
        testInvalidConfigurationAndDestructorBoundary();
        testLiveMotionHardSafetyCeilings();
        std::cout << "UNITREE_H2_DIRECT_API_CONTRACT_OK" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "UNITREE_H2_DIRECT_API_CONTRACT_FAILED: " << error.what()
                  << std::endl;
        return 1;
    }
}
