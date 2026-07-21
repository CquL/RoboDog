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

class UnitreeH2 : public RobotHardwareInterface
{
public:
    UnitreeH2() = default;
    explicit UnitreeH2(YAML::Node config);
    ~UnitreeH2() override;

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;

    int32_t readFsmId(int &fsm_id);
    int32_t readFsmMode(int &fsm_mode);
    int32_t readAvailableFsmIds(std::vector<int> &ids,
                                std::vector<std::string> &names);

private:
    int32_t verifyMotionFsmLocked();
    int32_t sendVelocityLocked(const RobotVelocityCommand &cmd);
    int32_t sendZeroVelocityLocked();
    int32_t runStateChangingActionLocked(const std::string &action);
    void watchdogLoop() noexcept;

    std::string network_interface_card_name_;
    int dds_domain_id_ = 0;
    float sdk_timeout_s_ = 10.0f;
    float velocity_command_duration_s_ = 0.3f;

    double max_vx_ = 0.2;
    double max_vy_ = 0.1;
    double max_omega_ = 0.3;

    int velocity_watchdog_hz_ = 50;
    int velocity_command_timeout_ms_ = 250;
    int velocity_zero_hold_ms_ = 1000;
    int required_motion_fsm_id_ = 601;

    bool verify_fsm_on_init_ = true;
    bool allow_motion_commands_ = false;
    bool allow_state_changing_actions_ = false;
    bool is_initialized_ = false;

    std::unique_ptr<unitree::robot::h2::LocoClient> loco_client_;
    std::atomic<bool> watchdog_running_{false};
    std::thread watchdog_thread_;
    std::mutex command_mutex_;
    // True as soon as a control RPC is attempted. A failed/timeout response
    // does not prove that the robot did not receive the command, so shutdown
    // must still make a best-effort StopMove call.
    bool control_rpc_attempted_ = false;
    bool velocity_command_active_ = false;
    std::chrono::steady_clock::time_point last_velocity_command_time_{};
};

#endif
