#ifndef UNITREE_H2_LIVE_MOTION_PLAN_H
#define UNITREE_H2_LIVE_MOTION_PLAN_H

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "robot_hardware_interface.h"

namespace unitree_h2_live_motion {

// Engineering stream-probe values, not vendor-rated limits. Each invocation
// sends exactly one bounded single-axis stream through
// RobotHardwareInterface::writeRobotVelocityCommand(). Axis signs remain
// uncalibrated until a protected on-robot test records physical motion.
constexpr double kDefaultLinearSpeed = 0.08;
constexpr double kDefaultYawSpeed = 0.08;
constexpr double kMinLinearSpeed = 0.01;
constexpr double kMinYawSpeed = 0.01;
constexpr double kMaxLinearSpeed = 0.10;
constexpr double kMaxYawSpeed = 0.15;

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

constexpr int kZeroHoldMilliseconds = 1000;
constexpr int kCountdownSeconds = 5;
constexpr int kInitialMotionFsmId = 601;

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

inline const std::array<const char *, 6> &axisNames()
{
    static const std::array<const char *, 6> names{
        "x-positive", "x-negative", "y-positive", "y-negative",
        "yaw-positive", "yaw-negative"};
    return names;
}

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

inline int nonZeroAxisCount(const RobotVelocityCommand &command)
{
    constexpr double epsilon = 1e-12;
    return (std::abs(command.vx) > epsilon ? 1 : 0) +
           (std::abs(command.vy) > epsilon ? 1 : 0) +
           (std::abs(command.omega) > epsilon ? 1 : 0);
}

} // namespace unitree_h2_live_motion

#endif
