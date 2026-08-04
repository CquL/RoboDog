// 旧版 Stage 06E 单轴实机探测计划的纯离线测试。
// 只验证数值包络、轴映射和时间顺序，不包含 SDK2，也不会发送机器人命令。
#include "unitree/unitree_h2_live_motion_plan.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string &message)
{
    // 用异常集中报告首个违反运动计划契约的条件。
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    using namespace unitree_h2_live_motion;

    try {
        // 默认速度/时长是已记录的历史探测契约，改变时必须同步评审实机门禁。
        require(std::abs(kDefaultLinearSpeed - 0.10) < 1e-12,
                "r9 default linear speed must be 0.10 m/s");
        require(kDefaultStreamMilliseconds == 1000,
                "r9 default stream must remain one second");

        for (const char *axis : axisNames()) {
            // 六种计划都必须存在、名称一致且只包含一个非零轴。
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

        // 核对默认计划的发送频率、RPC 次数、看门狗和厂商持续时间，保证脚本
        // 记录可以从计划字段确定性复现。
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

        // 合法的较短探测仍必须按 50 ms 周期整除并得到准确 RPC 次数。
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

        // 以下均为拒绝路径：超速、过长/过短、精度不合规、未标定方向词和
        // 多轴组合都不得生成可执行计划。
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
        // 时间关系要求：正常刷新 < 最大允许间隔 < 本地看门狗 < 厂商命令到期。
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
