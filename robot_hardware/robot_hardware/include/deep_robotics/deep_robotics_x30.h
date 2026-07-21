#ifndef DEEP_ROBOTICS_X30_H
#define DEEP_ROBOTICS_X30_H

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
    DeepRoboticsX30() {}
    DeepRoboticsX30(YAML::Node config);
    ~DeepRoboticsX30();

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;

private:
    enum class VelocityBackend {
        UDP_AXIS,
        UDP_PHYSICAL,
        UDP_NAVIGATION,
    };

    int32_t sendDatagram(const sockaddr_in &target_addr,
                         const void *data,
                         std::size_t size,
                         uint32_t code_for_log);
    int32_t sendSimpleCommand(const sockaddr_in &target_addr, uint32_t code, int32_t value = 0);
    int32_t sendMotionCommand(uint32_t code, int32_t value = 0);
    int32_t sendPerceptionCommand(uint32_t code, int32_t value = 0);
    int32_t sendAxisCommand(uint32_t code, int32_t axis_value);
    int32_t sendVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendAxisVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendPhysicalVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendNavigationVelocityCommand(const RobotVelocityCommand &cmd);
    int32_t sendStopCommand();
    int32_t sendGaitCommand(uint32_t gait_suffix, const std::string &gait_name);

    int32_t startHeartbeat();
    void heartbeatLoop();
    int32_t startVelocityWatchdog();
    void velocityWatchdogLoop();

    uint32_t motionCode(uint32_t suffix) const;
    std::string velocityBackendName() const;
    int32_t velocityToAxis(double velocity, double max_velocity, int32_t deadzone, bool invert) const;
    int32_t yawRateToAxis(double yaw_rate) const;

    std::string motion_ip_ = "192.168.1.103";
    int motion_port_ = 43893;

    std::string perception_ip_ = "192.168.1.105";
    int perception_port_ = 43899;

    std::string navigation_velocity_ip_ = "192.168.1.105";
    int navigation_velocity_port_ = 43897;

    int local_port_ = 0;
    int heartbeat_hz_ = 10;
    int velocity_watchdog_hz_ = 50;
    int velocity_command_timeout_ms_ = 250;
    int velocity_zero_hold_ms_ = 3000;

    double max_vx_ = 0.5;
    double max_vy_ = 0.3;
    double max_omega_ = 0.5;

    bool invert_vy_axis_ = true;
    bool invert_omega_axis_ = false;
    bool configure_non_manual_mode_ = true;
    bool configure_navigation_velocity_source_ = true;
    uint32_t motion_command_prefix_ = 0x31000000;
    std::string motion_command_source_ = "navigation";
    VelocityBackend velocity_backend_ = VelocityBackend::UDP_AXIS;

    int socket_fd_ = -1;
    sockaddr_in motion_addr_{};
    sockaddr_in perception_addr_{};
    sockaddr_in navigation_velocity_addr_{};

    bool is_initialized_ = false;
    std::atomic<bool> heartbeat_running_{false};
    std::thread heartbeat_thread_;
    std::atomic<bool> velocity_watchdog_running_{false};
    std::thread velocity_watchdog_thread_;
    std::mutex velocity_command_mutex_;
    bool velocity_command_active_ = false;
    std::chrono::steady_clock::time_point last_velocity_command_time_{};
};

#endif
