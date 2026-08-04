#ifndef UNITREE_H2_LIVE_MOTION_PLAN_H
#define UNITREE_H2_LIVE_MOTION_PLAN_H

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "robot_hardware_interface.h"

namespace unitree_h2_live_motion {

// Stage 06E 单轴实机探测所使用的工程参数，不是宇树公布的额定能力上限。
// 每次探测只生成一个轴向的 RobotVelocityCommand，实际发送仍通过抽象接口
// RobotHardwareInterface::writeRobotVelocityCommand() 完成。
//
// 这里的 positive/negative 是数学坐标符号；在保护架实测并记录物理方向前，
// 不能直接把它解释为“前/后/左/右”。
//
// 此旧版 Stage 06E 计划固定保留 0.10 m/s 的探测上限，因此 0.90 m/s 会被拒绝。
// 它与当前生产适配器 YAML 中更大的项目限幅是两个不同层次的约束。
constexpr double kDefaultLinearSpeed = 0.10;
constexpr double kDefaultYawSpeed = 0.08;
constexpr double kMinLinearSpeed = 0.01;
constexpr double kMinYawSpeed = 0.01;
constexpr double kMaxLinearSpeed = 0.10;
constexpr double kMaxYawSpeed = 0.15;

// 速度流的时间结构：每 50 ms（20 Hz）刷新一次，并要求发送间隔、软件看门狗
// 和厂商命令有效期依次增大，保证上层断流时先由本项目触发停止。
constexpr int kDefaultStreamMilliseconds = 1000;
constexpr int kMinStreamMilliseconds = 250;
constexpr int kMaxStreamMilliseconds = 1000;
constexpr int kStreamStepMilliseconds = 50;
constexpr int kCommandHz = 20;
constexpr int kCommandPeriodMilliseconds = 50;
constexpr int kMaxSendGapMilliseconds = 100;
constexpr int kWatchdogTimeoutMilliseconds = 150;
constexpr float kVendorCommandDurationS = 0.30f;
constexpr float kSdkTimeoutS = 0.20f;

// 实机脚本使用的附加安全参数：停止保持时长、人工倒计时和初始 FSM 门禁。
constexpr int kZeroHoldMilliseconds = 1000;
constexpr int kCountdownSeconds = 5;
constexpr int kInitialMotionFsmId = 601;

// 一次单轴探测的完整、不可歧义描述。command 是最终送入抽象 HAL 的速度；
// expected_rpc_count 供测试核对实际发送次数，其余字段记录安全时间包络。
struct AxisPlan {
    std::string axis;
    RobotVelocityCommand command;
    double linear_speed = kDefaultLinearSpeed;
    double yaw_speed = kDefaultYawSpeed;
    int stream_ms = kDefaultStreamMilliseconds;
    int command_hz = kCommandHz;
    int command_period_ms = kCommandPeriodMilliseconds;
    int max_send_gap_ms = kMaxSendGapMilliseconds;
    int expected_rpc_count =
        kDefaultStreamMilliseconds / kCommandPeriodMilliseconds;
    int watchdog_ms = kWatchdogTimeoutMilliseconds;
    float vendor_duration_s = kVendorCommandDurationS;
};

// 返回允许的六个“坐标轴+符号”名称。故意不接受 forward/left 等尚未标定的
// 人类方向词，也不提供多轴组合名称。
inline const std::array<const char *, 6> &axisNames()
{
    static const std::array<const char *, 6> names{
        "x-positive", "x-negative", "y-positive", "y-negative",
        "yaw-positive", "yaw-negative"};
    return names;
}

// 要求速度值最多精确到 0.001，避免无法在测试记录中稳定复现的细小改动。
inline bool hasMillistepPrecision(double value)
{
    return std::abs(value * 1000.0 - std::round(value * 1000.0)) < 1e-9;
}

inline bool streamWithinLimits(int stream_ms)
{
    return stream_ms >= kMinStreamMilliseconds &&
           stream_ms <= kMaxStreamMilliseconds &&
           stream_ms % kStreamStepMilliseconds == 0 &&
           kCommandHz > 0 && kCommandPeriodMilliseconds > 0 &&
           1000 / kCommandHz == kCommandPeriodMilliseconds &&
           stream_ms % kCommandPeriodMilliseconds == 0 &&
           kCommandPeriodMilliseconds < kMaxSendGapMilliseconds &&
           kMaxSendGapMilliseconds < kWatchdogTimeoutMilliseconds &&
           kWatchdogTimeoutMilliseconds <
               static_cast<int>(kVendorCommandDurationS * 1000.0f);
}

// 同时验证有限数、速度上下限、记录精度和完整的刷新/超时关系。
inline bool profileWithinLimits(double linear_speed, double yaw_speed,
                                int stream_ms)
{
    return std::isfinite(linear_speed) && std::isfinite(yaw_speed) &&
           hasMillistepPrecision(linear_speed) &&
           hasMillistepPrecision(yaw_speed) &&
           linear_speed >= kMinLinearSpeed &&
           linear_speed <= kMaxLinearSpeed &&
           yaw_speed >= kMinYawSpeed && yaw_speed <= kMaxYawSpeed &&
           streamWithinLimits(stream_ms);
}

// 根据名称生成单轴速度计划。任何不受支持的名称或超出安全包络的参数均返回
// std::nullopt，调用方不得把无效计划降级成其他运动。
inline std::optional<AxisPlan> planForAxis(
    const std::string &axis,
    double linear_speed = kDefaultLinearSpeed,
    double yaw_speed = kDefaultYawSpeed,
    int stream_ms = kDefaultStreamMilliseconds)
{
    if (!profileWithinLimits(linear_speed, yaw_speed, stream_ms)) {
        return std::nullopt;
    }

    const int expected_rpc_count =
        stream_ms / kCommandPeriodMilliseconds;

    // 所有合法分支共用相同的时间包络，只改变速度向量的轴和符号。
    const auto make_plan = [&](const RobotVelocityCommand &command) {
        return AxisPlan{axis,
                        command,
                        linear_speed,
                        yaw_speed,
                        stream_ms,
                        kCommandHz,
                        kCommandPeriodMilliseconds,
                        kMaxSendGapMilliseconds,
                        expected_rpc_count,
                        kWatchdogTimeoutMilliseconds,
                        kVendorCommandDurationS};
    };

    if (axis == "x-positive") {
        return make_plan({linear_speed, 0.0, 0.0});
    }
    if (axis == "x-negative") {
        return make_plan({-linear_speed, 0.0, 0.0});
    }
    if (axis == "y-positive") {
        return make_plan({0.0, linear_speed, 0.0});
    }
    if (axis == "y-negative") {
        return make_plan({0.0, -linear_speed, 0.0});
    }
    if (axis == "yaw-positive") {
        return make_plan({0.0, 0.0, yaw_speed});
    }
    if (axis == "yaw-negative") {
        return make_plan({0.0, 0.0, -yaw_speed});
    }
    return std::nullopt;
}

// 测试辅助函数：验证生成的命令恰好只有一个非零轴，防止探测过程混入组合运动。
inline int nonZeroAxisCount(const RobotVelocityCommand &command)
{
    constexpr double epsilon = 1e-12;
    return (std::abs(command.vx) > epsilon ? 1 : 0) +
           (std::abs(command.vy) > epsilon ? 1 : 0) +
           (std::abs(command.omega) > epsilon ? 1 : 0);
}

} // namespace unitree_h2_live_motion

#endif
