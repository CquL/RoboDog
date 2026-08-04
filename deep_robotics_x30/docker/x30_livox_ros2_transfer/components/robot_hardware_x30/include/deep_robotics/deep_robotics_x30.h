#ifndef DEEP_ROBOTICS_X30_H
#define DEEP_ROBOTICS_X30_H

// X30 对厂商无关接口 RobotHardwareInterface 的实现。
//
// 控制分布在三个厂商 UDP 端点：
//   192.168.1.103:43893 - 运动主机动作、heartbeat 和直接速度
//   192.168.1.105:43899 - 感知主机模式与速度源选择
//   192.168.1.105:43897 - 导航速度包输入
// 本类只发送命令；状态确认由外部 ROS1 系统提供，不纳入 HAL 接口。

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>
#include "robot_hardware_interface.h"

class DeepRoboticsX30 : public RobotHardwareInterface
{
public:
    // 构造阶段只读取配置，不执行网络 I/O。
    DeepRoboticsX30() {}
    DeepRoboticsX30(YAML::Node config);

    // 析构时先停止后台线程，再关闭共享 socket。
    ~DeepRoboticsX30();

    // 打开 UDP 传输、建立厂商会话并启动安全线程。
    int32_t initRobotHardware() override;

    // 限制机身坐标速度，按所选后端编码，并刷新 watchdog 截止时间。
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;

    // 将通用动作名转换为一条 X30 运动主机命令。
    int32_t writeActionCommand(std::string action) override;

private:
    // 三种后端的数值和目的端含义不同，必须明确区分。UDP_AXIS 模拟手柄，
    // UDP_PHYSICAL 以千分单位直发运动主机，UDP_NAVIGATION 通过感知/导航链
    // 发送 SI 单位 double。
    enum class VelocityBackend {
        UDP_AXIS,
        UDP_PHYSICAL,
        UDP_NAVIGATION,
    };

    // 统一处理发送，使各后端共用 socket 状态检查、完整报文校验和错误报告。
    int32_t sendDatagram(const sockaddr_in &target_addr,
                         const void *data,
                         std::size_t size,
                         uint32_t code_for_log);

    // 这些封装只选择目的端，字节级编码由 x30_udp_protocol 负责。
    int32_t sendSimpleCommand(const sockaddr_in &target_addr, uint32_t code, int32_t value = 0);
    int32_t sendMotionCommand(uint32_t code, int32_t value = 0);
    int32_t sendPerceptionCommand(uint32_t code, int32_t value = 0);
    int32_t sendAxisCommand(uint32_t code, int32_t axis_value);

    // 将一条通用速度命令路由到配置的厂商后端。
    int32_t sendVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendAxisVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendPhysicalVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendNavigationVelocityCommand(const RobotVelocityCommand &cmd);

    // 停止和步态操作与速度写入串行化。切换步态前重复发送零速度，避免旧速度
    // 与切换过程重叠。
    int32_t sendStopCommand();
    int32_t sendGaitCommand(uint32_t gait_suffix, const std::string &gait_name);

    // 厂商会话需要独立于上层控制循环的周期 heartbeat。
    int32_t startHeartbeat();
    void heartbeatLoop();

    // watchdog 在非零命令流超时后，于限定时间内重复补发零速度。
    int32_t startVelocityWatchdog();
    void velocityWatchdogLoop();

    // 协议辅助函数组合命令源前缀；使用轴后端时，再将 SI 单位映射为控制轴值。
    uint32_t motionCode(uint32_t suffix) const;
    std::string velocityBackendName() const;
    int32_t velocityToAxis(double velocity, double max_velocity, int32_t deadzone, bool invert) const;
    int32_t yawRateToAxis(double yaw_rate) const;

    // 厂商端点配置。默认值对应 X30 内网，也可通过 YAML 覆盖以便诊断和迁移。
    std::string motion_ip_ = "192.168.1.103";
    int motion_port_ = 43893;

    std::string perception_ip_ = "192.168.1.105";
    int perception_port_ = 43899;

    std::string navigation_velocity_ip_ = "192.168.1.105";
    int navigation_velocity_port_ = 43897;

    // 会话与命令超时参数。local_port 为 0 时由内核选择源端口，正值用于绑定
    // 固定诊断端口。
    int local_port_ = 0;
    int heartbeat_hz_ = 10;
    int velocity_watchdog_hz_ = 50;
    int velocity_command_timeout_ms_ = 250;
    int velocity_zero_hold_ms_ = 3000;

    // 项目级 SI 单位命令范围。普通楼梯模式可进一步收紧，但不改这些配置上限。
    double max_vx_ = 0.5;
    double max_vy_ = 0.3;
    double max_omega_ = 0.5;

    // 命令源选项决定由哪条原厂链路接收数据；轴反向只作用于手柄兼容编码。
    bool invert_vy_axis_ = true;
    bool invert_omega_axis_ = false;
    bool configure_non_manual_mode_ = true;
    bool configure_navigation_velocity_source_ = true;
    uint32_t motion_command_prefix_ = 0x31000000;
    std::string motion_command_source_ = "navigation";
    VelocityBackend velocity_backend_ = VelocityBackend::UDP_AXIS;

    // 一个 socket 服务全部 UDP 目的端；sockaddr 在初始化时生成，随后由前台
    // 和后台线程复用。
    int socket_fd_ = -1;
    sockaddr_in motion_addr_{};
    sockaddr_in perception_addr_{};
    sockaddr_in navigation_velocity_addr_{};

    // 线程标志使用原子变量，速度状态由同一 mutex 保护，防止步态切换、前台
    // 写入和 watchdog 补零相互穿插。
    bool is_initialized_ = false;
    std::atomic<bool> heartbeat_running_{false};
    std::thread heartbeat_thread_;
    std::atomic<bool> velocity_watchdog_running_{false};
    std::thread velocity_watchdog_thread_;
    std::mutex velocity_command_mutex_;
    bool ordinary_stairs_limits_active_ = false;
    bool velocity_command_active_ = false;
    std::chrono::steady_clock::time_point last_velocity_command_time_{};
};

#endif
