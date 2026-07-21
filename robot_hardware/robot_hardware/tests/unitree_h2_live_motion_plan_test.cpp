#include "unitree/unitree_h2_live_motion_plan.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    using namespace unitree_h2_live_motion;
    try {
        for (const char *axis : axisNames()) {
            const auto plan = planForAxis(axis);
            require(plan.has_value(), std::string("missing plan for ") + axis);
            require(plan->axis == axis, std::string("wrong axis name for ") + axis);
            require(nonZeroAxisCount(plan->command) == 1,
                    std::string("plan is not single-axis: ") + axis);
            require(std::abs(plan->command.vx) <= kDefaultLinearSpeed &&
                        std::abs(plan->command.vy) <= kDefaultLinearSpeed &&
                        std::abs(plan->command.omega) <= kDefaultYawSpeed,
                    std::string("plan exceeds probe ceiling: ") + axis);
        }
        require(planForAxis("x-positive")->command.vx == kDefaultLinearSpeed &&
                    planForAxis("x-positive")->command.vy == 0.0 &&
                    planForAxis("x-positive")->command.omega == 0.0,
                "x-positive vector mapping changed");
        require(planForAxis("x-negative")->command.vx == -kDefaultLinearSpeed,
                "x-negative vector mapping changed");
        require(planForAxis("y-positive")->command.vy == kDefaultLinearSpeed &&
                    planForAxis("y-negative")->command.vy == -kDefaultLinearSpeed,
                "y-axis vector mapping changed");
        require(planForAxis("yaw-positive")->command.omega == kDefaultYawSpeed &&
                    planForAxis("yaw-negative")->command.omega == -kDefaultYawSpeed,
                "yaw-axis vector mapping changed");
        const auto stream_plan =
            planForAxis("x-positive", 0.08, 0.08, 1000);
        require(stream_plan.has_value(), "bounded stream profile rejected");
        require(stream_plan->command.vx == 0.08 &&
                    stream_plan->stream_ms == 1000 &&
                    stream_plan->command_hz == 20 &&
                    stream_plan->command_period_ms == 50 &&
                    stream_plan->max_send_gap_ms == 100 &&
                    stream_plan->expected_rpc_count == 20 &&
                    stream_plan->watchdog_ms == 150 &&
                    std::abs(stream_plan->vendor_duration_s - 0.30f) < 1e-6f,
                "bounded stream profile changed");
        require(stream_plan->command_period_ms <
                    stream_plan->max_send_gap_ms &&
                    stream_plan->max_send_gap_ms < stream_plan->watchdog_ms &&
                    stream_plan->watchdog_ms <
                        static_cast<int>(
                            stream_plan->vendor_duration_s * 1000.0f),
                "bounded stream timing order invalid");
        const auto shortest_plan =
            planForAxis("x-positive", 0.08, 0.08, 250);
        require(shortest_plan.has_value() &&
                    shortest_plan->expected_rpc_count == 5,
                "shortest bounded stream profile changed");
        require(!planForAxis("x-positive", kMaxLinearSpeed + 0.001,
                             kDefaultYawSpeed, 1000)
                     .has_value(),
                "excessive linear speed was accepted");
        require(!planForAxis("yaw-positive", kDefaultLinearSpeed,
                             kMaxYawSpeed + 0.001, 1000)
                     .has_value(),
                "excessive yaw speed was accepted");
        require(!planForAxis("x-positive", kDefaultLinearSpeed,
                             kDefaultYawSpeed,
                             kMaxStreamMilliseconds +
                                 kStreamStepMilliseconds)
                     .has_value(),
                "excessive stream duration was accepted");
        require(!planForAxis("x-positive", kMinLinearSpeed - 0.001,
                             kDefaultYawSpeed, 1000)
                     .has_value(),
                "too-small linear speed was accepted");
        require(!planForAxis("x-positive", 0.0805,
                             kDefaultYawSpeed, 1000)
                     .has_value(),
                "unlogged sub-millistep linear speed was accepted");
        require(!planForAxis("yaw-positive", kDefaultLinearSpeed,
                             kMinYawSpeed - 0.001, 1000)
                     .has_value(),
                "too-small yaw speed was accepted");
        require(!planForAxis("x-positive", kDefaultLinearSpeed,
                             kDefaultYawSpeed,
                             kMinStreamMilliseconds -
                                 kStreamStepMilliseconds)
                     .has_value(),
                "too-short stream duration was accepted");
        require(!planForAxis("x-positive", kDefaultLinearSpeed,
                             kDefaultYawSpeed, 275)
                     .has_value(),
                "non-50-ms stream duration was accepted");
        require(!planForAxis("forward").has_value(),
                "uncalibrated human direction name was accepted");
        require(!planForAxis("x-positive+y-positive").has_value(),
                "combined-axis plan was accepted");
        require(kCommandPeriodMilliseconds < kMaxSendGapMilliseconds &&
                    kMaxSendGapMilliseconds <
                        kWatchdogTimeoutMilliseconds,
                "stream refresh/watchdog order changed");
        require(kWatchdogTimeoutMilliseconds <
                    static_cast<int>(kVendorCommandDurationS * 1000.0f),
                "watchdog must precede vendor command expiry");
        require(planForAxis("x-positive")->watchdog_ms ==
                    kWatchdogTimeoutMilliseconds &&
                    std::abs(planForAxis("x-positive")->vendor_duration_s -
                             kVendorCommandDurationS) < 1e-6f &&
                    planForAxis("x-positive")->stream_ms ==
                        kDefaultStreamMilliseconds &&
                    planForAxis("x-positive")->expected_rpc_count == 20,
                "default r8 stream contract changed");
        require(kSdkTimeoutS > 0.0f && kSdkTimeoutS < kVendorCommandDurationS,
                "SDK timeout must be bounded below vendor duration");
        require(kInitialMotionFsmId == 601,
                "initial live-motion FSM gate changed without review");
        std::cout << "UNITREE_H2_LIVE_MOTION_PLAN_OK" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "UNITREE_H2_LIVE_MOTION_PLAN_FAILED: " << error.what()
                  << std::endl;
        return 1;
    }
}
