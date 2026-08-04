// UnitreeH2 HAL 的离线直接 API 契约测试。
//
// CMake 会让本文件和 unitree_h2.cpp 优先包含 tests/fakes 下的假 SDK2，
// 因此测试不会创建真实 DDS 通道，也不会向实机发送命令。Recorder 用于断言
// 抽象接口的输入被正确映射到 LocoClient，并覆盖错误、异常和看门狗路径。
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

// 轻量断言：失败时抛出带场景说明的异常，由 main 统一输出测试结果。
void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(float lhs, float rhs)
{
    // SDK2 速度参数为 float，比较时使用固定容差而非直接比较计算结果。
    return std::abs(lhs - rhs) < 1e-6f;
}

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout)
{
    // 等待后台看门狗产生可观察的 Recorder 变化；2 ms 轮询只用于离线测试。
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
    // 生成确定性的假网卡/Domain/超时配置。两个参数分别控制非零速度和状态动作
    // 门禁；默认保持只读/停止可用的安全状态。
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
    // 验证初始化参数透传、重复初始化幂等，以及默认配置拒绝所有非零速度和
    // 状态动作，同时仍允许零速度与 StopMove。
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
    // 任意非零值都应触发门禁，不能用 epsilon 把微小运动当成零速度。
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
    // 打开测试许可后，验证速度逐轴限幅及所有受支持动作到 SDK2 RPC 的唯一映射。
    // 每个状态动作还必须先成功调用 StopMove。
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
    // H2 的 lie_down 不得猜测映射为 Damp；未知动作也不得接触 SDK。
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
    // 将假 SDK 配置为返回非零码或抛异常，确认 HAL 始终把它们转换为 int32
    // 项目错误码，并在“命令可能已投递”的路径立即 StopMove/保留看门狗重试。
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
    // fake 在记录 SetVelocity 后才抛异常，模拟机器人可能已经收包、但客户端
    // 未收到成功响应的安全关键不确定投递场景。
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
    // 首次 StopMove 抛异常时 active 状态不能清除；解除故障后看门狗必须继续
    // 重试并最终记录一次成功停止。
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
    // 覆盖连接就绪读取、ChannelFactory/LocoClient 初始化，以及三个只读 getter
    // 的厂商错误码/异常归一化。所有失败都不得穿透抽象接口异常边界。
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
    // 关闭初始化 FSM 读取仅允许在 motion=false 时使用；对象初始化后仍可显式
    // 调用 getter 获取状态。
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
    // 非有限配置必须在 ChannelFactory 前被拒绝；析构阶段 StopMove 抛异常也不能
    // 从 noexcept 的资源清理边界逃逸。
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
    // 验证当前项目硬上限 1.00/0.10/0.70 和 0.30 s：等于上限可配置，超过任一
    // 项即在 SDK 初始化前失败。这里不把这些数值声明为宇树官方额定上限。
    Recorder::Reset();
    {
        YAML::Node accepted_velocity = makeConfig(true, false);
        accepted_velocity["max_vx"] = 1.00;
        accepted_velocity["max_omega"] = 0.70;
        UnitreeH2 accepted_robot(accepted_velocity);
        require(accepted_robot.initRobotHardware() == CMD_SUCCESS,
                "configured project velocity ceilings were rejected");
        RobotVelocityCommand command{2.0, 0.0, 2.0};
        require(accepted_robot.writeRobotVelocityCommand(command) == CMD_SUCCESS,
                "configured project velocity command failed");
        require(near(Recorder::last_vx, 1.00f) &&
                    near(Recorder::last_omega, 0.70f),
                "project velocity ceilings were not applied to the SDK command");
    }

    Recorder::Reset();
    YAML::Node excessive_velocity = makeConfig(true, false);
    excessive_velocity["max_vx"] = 1.01;
    UnitreeH2 velocity_robot(excessive_velocity);
    require(velocity_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "configuration above the absolute vx ceiling was accepted");
    require(Recorder::channel_init_calls == 0,
            "excessive vx configuration reached ChannelFactory");

    Recorder::Reset();
    YAML::Node excessive_omega = makeConfig(true, false);
    excessive_omega["max_omega"] = 0.71;
    UnitreeH2 omega_robot(excessive_omega);
    require(omega_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "configuration above the absolute omega ceiling was accepted");
    require(Recorder::channel_init_calls == 0,
            "excessive omega configuration reached ChannelFactory");

    Recorder::Reset();
    YAML::Node excessive_duration = makeConfig(true, false);
    excessive_duration["velocity_command_duration_s"] = 0.31f;
    UnitreeH2 duration_robot(excessive_duration);
    require(duration_robot.initRobotHardware() == ERROR_ROBOT_HARDWARE_INIT,
            "configuration above the absolute duration ceiling was accepted");

    Recorder::Reset();
    YAML::Node no_fsm_gate = makeConfig(true, false);
    no_fsm_gate["verify_fsm_on_init"] = false;
    // 非零运动许可与初始化 FSM 门禁必须绑定，配置不能只开前者。
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
    // 即使初始化时为 601，运行中状态改变后下一条非零速度仍必须被逐命令门禁
    // 拒绝，并主动请求 StopMove。
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
        // 按初始化/正常映射/故障/就绪/配置/硬上限分组执行，任何 require 失败
        // 都终止并给 CTest 返回非零。
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
