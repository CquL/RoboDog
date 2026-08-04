#ifndef UNITREE_H2_H
#define UNITREE_H2_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/h2/loco/h2_loco_client.hpp>

#include "robot_hardware_interface.h"

// Unitree H2 高层运动控制适配器。
//
// 上层只依赖 RobotHardwareInterface 的三个统一入口，本类负责把它们映射为
// Unitree SDK2 的 H2 LocoClient RPC：
//   initRobotHardware()             -> 初始化 DDS/SDK，并读取 FSM 就绪状态；
//   writeRobotVelocityCommand(cmd)  -> LocoClient::SetVelocity(...)；
//   writeActionCommand(action)      -> StopMove/StandUp/Start/Damp/Squat/Sit。
//
// 注意：SDK2 的 RPC 底层仍使用 DDS 与机器人运动服务通信，但上层算法不需要
// 直接发布或订阅原厂 ROS 2 控制话题。
class UnitreeH2 : public RobotHardwareInterface
{
public:
    // 默认构造只用于需要稍后配置的场景；正常运行由 RobotFactory 使用 YAML
    // 构造函数创建对象。
    UnitreeH2() = default;
    explicit UnitreeH2(YAML::Node config);

    // 析构时停止看门狗，并在曾尝试发送控制 RPC 时尽力调用 StopMove。
    // 析构阶段不能向调用方返回错误，因此失败只记录日志。
    ~UnitreeH2() override;

    // 初始化 DDS ChannelFactory 和 LocoClient。成功返回 CMD_SUCCESS；配置非法、
    // SDK 初始化失败或运动 FSM 未就绪时返回项目统一错误码。
    int32_t initRobotHardware() override;

    // 输入为项目统一的机体速度（vx/vy: m/s，omega: rad/s）。函数会检查有限
    // 数、按 YAML 限幅、执行运动许可和 FSM 门禁，再调用 SetVelocity。
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;

    // 输入为 robot_hardware_constant.h 中的动作字符串。StopMove 始终可请求；
    // 其余改变状态的动作还必须通过 allow_state_changing_actions 安全门禁。
    int32_t writeActionCommand(std::string action) override;

    // 以下只读 getter 供测试/诊断读取原厂运动 FSM。引用参数承接 SDK 输出，
    // 返回值仍转换为项目统一错误码，不把厂商异常抛给上层。
    int32_t readFsmId(int &fsm_id);
    int32_t readFsmMode(int &fsm_mode);
    int32_t readAvailableFsmIds(std::vector<int> &ids,
                                std::vector<std::string> &names);

private:
    // 下列 *Locked 函数要求调用者已持有 command_mutex_，从而保证测试线程、
    // 上层控制线程和看门狗不会并发写入 LocoClient。
    int32_t verifyMotionFsmLocked();
    int32_t sendVelocityLocked(const RobotVelocityCommand &cmd);
    int32_t sendZeroVelocityLocked();
    int32_t runStateChangingActionLocked(const std::string &action);

    // 独立安全线程：非零命令在规定时间内未刷新时，在整个零速度保持窗口内
    // 重复调用 StopMove；窗口结束后还需一次成功调用才清除活动状态。
    void watchdogLoop() noexcept;

    // DDS/SDK 通信参数。网卡名必须是通往 H2 原厂运动网络的接口。
    std::string network_interface_card_name_;
    int dds_domain_id_ = 0;
    float sdk_timeout_s_ = 10.0f;

    // SetVelocity 的厂商持续时间参数，不等于上层控制循环的总运动时长。
    float velocity_command_duration_s_ = 0.3f;

    // YAML 可配置的软件限幅；initRobotHardware() 还会用 cpp 中的项目硬上限
    // 再校验一次，避免配置文件意外放宽安全包络。
    double max_vx_ = 0.2;
    double max_vy_ = 0.1;
    double max_omega_ = 0.3;

    // 看门狗刷新频率、命令过期阈值和 StopMove 保持时间，单位分别为 Hz/ms/ms。
    int velocity_watchdog_hz_ = 50;
    int velocity_command_timeout_ms_ = 250;
    int velocity_zero_hold_ms_ = 1000;

    // 非零速度必须匹配的高层运动 FSM；当前实机验证值为 601。
    int required_motion_fsm_id_ = 601;

    // 三层许可状态：初始化是否读 FSM、是否允许非零速度、是否允许状态动作。
    // 默认均按安全方向设置，只有显式 YAML 配置才会放开运动/动作。
    bool verify_fsm_on_init_ = true;
    bool allow_motion_commands_ = false;
    bool allow_state_changing_actions_ = false;
    bool is_initialized_ = false;

    // SDK 客户端及看门狗生命周期。
    std::unique_ptr<unitree::robot::h2::LocoClient> loco_client_;
    std::atomic<bool> watchdog_running_{false};
    std::thread watchdog_thread_;

    // 串行化所有 LocoClient 状态读取和控制 RPC。
    std::mutex command_mutex_;

    // True as soon as a control RPC is attempted. A failed/timeout response
    // does not prove that the robot did not receive the command, so shutdown
    // must still make a best-effort StopMove call.
    bool control_rpc_attempted_ = false;

    // 记录非零速度流是否仍处于活动状态及最后刷新时刻，供看门狗判断超时。
    bool velocity_command_active_ = false;
    std::chrono::steady_clock::time_point last_velocity_command_time_{};
};

#endif
