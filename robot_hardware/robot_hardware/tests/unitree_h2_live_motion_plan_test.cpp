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
        require(std::abs(kDefaultLinearSpeed - 0.10) < 1e-12,
                "r9 default linear speed must be 0.10 m/s");
        require(kDefaultStreamMilliseconds == 1000,
                "r9 default stream must remain one second");

        for (const char *axis : axisNames()) {
            const auto plan = planForAxis(axis);
            require(plan.has_value(), std::string("missing plan for ") + axis);
            require(plan->axis == axis,
                    std::string("wrong axis name for ") + axis);
            require(nonZeroAxisCount(plan->command) == 1,
                    std::string("plan is not single-axis: ") + axis);
            require(std::abs(plan->command.vx) <= kDefaultLinearSpeed &&
                        std::abs(plan->command.vy) <= kDefaultLinearSpeed &&
                        std::abs(plan->command.omega) <= kDefaultYawSpeed,
                    std::string("plan exceeds probe ceiling: ") + axis);
        }

        require(planForAxis("x-positive")->command.vx ==
                    kDefaultLinearSpeed &&
                    planForAxis("x-positive")->command.vy == 0.0 &&
                    planForAxis("x-positive")->command.omega == 0.0,
                "x-positive vector mapping changed");
        require(planForAxis("x-negative")->command.vx ==
                    -kDefaultLinearSpeed,
                "x-negative vector mapping changed");
        require(planForAxis("y-positive")->command.vy ==
                    kDefaultLinearSpeed &&
                    planForAxis("y-negative")->command.vy ==
                    -kDefaultLinearSpeed,
                "y-axis vector mapping changed");
        require(planForAxis("yaw-positive")->command.omega ==
                    kDefaultYawSpeed &&
                    planForAxis("yaw-negative")->command.omega ==
                    -kDefaultYawSpeed,
                "yaw-axis vector mapping changed");

        const auto default_plan = planForAxis("x-positive");
        require(default_plan.has_value(), "default r9 plan rejected");
        require(default_plan->command.vx == 0.10 &&
                    default_plan->stream_ms == 1000 &&
                    default_plan->command_hz == 20 &&
                    default_plan->command_period_ms == 50 &&
                    default_plan->max_send_gap_ms == 100 &&
                    default_plan->expected_rpc_count == 20 &&
                    default_plan->watchdog_ms == 150 &&
                    std::abs(default_plan->vendor_duration_s - 0.30f) < 1e-6f,
                "default r9 stream profile changed");

        require(default_plan->command_period_ms <
                    default_plan->max_send_gap_ms &&
                    default_plan->max_send_gap_ms < default_plan->watchdog_ms &&
                    default_plan->watchdog_ms <
                        static_cast<int>(
                            default_plan->vendor_duration_s * 1000.0f),
                "bounded stream timing order invalid");

        const auto shortest_plan =
            planForAxis("x-positive", 0.10, 0.08, 250);
        require(shortest_plan.has_value() &&
                    shortest_plan->expected_rpc_count == 5,
                "shortest bounded stream profile changed");

        const auto half_second_plan =
            planForAxis("x-positive", 0.10, 0.08, 500);
        require(half_second_plan.has_value() &&
                    half_second_plan->expected_rpc_count == 10,
                "half-second diagnostic profile changed");

        require(!planForAxis("x-positive", 0.90, 0.08, 1000).has_value(),
                "0.90 m/s must remain rejected in Stage 06E");
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
        require(!planForAxis("x-positive", 0.1005,
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
        require(kSdkTimeoutS > 0.0f &&
                    kSdkTimeoutS < kVendorCommandDurationS,
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
