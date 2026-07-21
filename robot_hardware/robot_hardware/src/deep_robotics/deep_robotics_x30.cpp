#include "deep_robotics_x30.h"
#include "x30_udp_protocol.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr uint32_t X30_HEARTBEAT_SUFFIX = 0x00040001;
constexpr uint32_t X30_CONFIRM_CONNECTION_SUFFIX = 0x00020001;
constexpr uint32_t X30_NON_MANUAL_MODE_SUFFIX = 0x00010C03;
constexpr uint32_t X30_VELOCITY_SOURCE = 0x3101EE03;
constexpr uint32_t X30_STAND_UP_LIE_DOWN_SUFFIX = 0x00010202;
constexpr uint32_t X30_START_STOP_MOTION_SUFFIX = 0x00010201;

constexpr uint32_t X30_AXIS_VX_SUFFIX = 0x00010130;
constexpr uint32_t X30_AXIS_VY_SUFFIX = 0x00010131;
constexpr uint32_t X30_AXIS_YAW_SUFFIX = 0x00010135;

constexpr int32_t X30_AXIS_MAX = 32767;
constexpr int32_t X30_VX_DEADZONE = 6553;
constexpr int32_t X30_VY_DEADZONE = 24576;
constexpr int32_t X30_YAW_DEADZONE = 28212;
constexpr double X30_ZERO_VELOCITY_EPSILON = 1e-9;

template <typename T>
T readYaml(const YAML::Node &node, const std::string &key, const T &default_value)
{
    if (node && node[key]) {
        return node[key].as<T>();
    }
    return default_value;
}

sockaddr_in makeUdpAddress(const std::string &ip, int port)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    return addr;
}

} // namespace

DeepRoboticsX30::DeepRoboticsX30(YAML::Node config) : RobotHardwareInterface(config)
{
    motion_ip_ = readYaml<std::string>(config, "motion_ip", motion_ip_);
    motion_port_ = readYaml<int>(config, "motion_port", motion_port_);

    perception_ip_ = readYaml<std::string>(config, "perception_ip", perception_ip_);
    perception_port_ = readYaml<int>(config, "perception_port", perception_port_);

    navigation_velocity_ip_ =
        readYaml<std::string>(config, "navigation_velocity_ip", navigation_velocity_ip_);
    navigation_velocity_port_ =
        readYaml<int>(config, "navigation_velocity_port", navigation_velocity_port_);

    local_port_ = readYaml<int>(config, "local_port", local_port_);
    heartbeat_hz_ = readYaml<int>(config, "heartbeat_hz", heartbeat_hz_);
    velocity_watchdog_hz_ =
        readYaml<int>(config, "velocity_watchdog_hz", velocity_watchdog_hz_);
    velocity_command_timeout_ms_ = readYaml<int>(
        config, "velocity_command_timeout_ms", velocity_command_timeout_ms_);
    velocity_zero_hold_ms_ =
        readYaml<int>(config, "velocity_zero_hold_ms", velocity_zero_hold_ms_);

    max_vx_ = readYaml<double>(config, "max_vx", max_vx_);
    max_vy_ = readYaml<double>(config, "max_vy", max_vy_);
    max_omega_ = readYaml<double>(config, "max_omega", max_omega_);

    invert_vy_axis_ = readYaml<bool>(config, "invert_vy_axis", invert_vy_axis_);
    invert_omega_axis_ = readYaml<bool>(config, "invert_omega_axis", invert_omega_axis_);

    configure_non_manual_mode_ =
        readYaml<bool>(config, "configure_non_manual_mode", configure_non_manual_mode_);

    configure_navigation_velocity_source_ =
        readYaml<bool>(config, "configure_navigation_velocity_source", configure_navigation_velocity_source_);

    const std::string velocity_backend =
        readYaml<std::string>(config, "velocity_backend", "udp_axis");
    if (velocity_backend == "udp_axis") {
        velocity_backend_ = VelocityBackend::UDP_AXIS;
    } else if (velocity_backend == "udp_physical") {
        velocity_backend_ = VelocityBackend::UDP_PHYSICAL;
    } else if (velocity_backend == "udp_navigation") {
        velocity_backend_ = VelocityBackend::UDP_NAVIGATION;
    } else {
        std::cerr << "[DeepRoboticsX30] Unknown velocity_backend: "
                  << velocity_backend
                  << ". Use udp_axis, udp_physical, or udp_navigation. "
                  << "Falling back to udp_axis."
                  << std::endl;
        velocity_backend_ = VelocityBackend::UDP_AXIS;
    }

    motion_command_source_ = readYaml<std::string>(config, "motion_command_source", motion_command_source_);
    if (motion_command_source_ == "remote") {
        motion_command_prefix_ = x30_udp_protocol::kRemoteCommandPrefix;
    } else if (motion_command_source_ == "navigation") {
        motion_command_prefix_ = x30_udp_protocol::kNavigationCommandPrefix;
    } else {
        std::cerr << "[DeepRoboticsX30] Unknown motion_command_source: "
                  << motion_command_source_
                  << ". Use navigation or remote. Falling back to navigation."
                  << std::endl;
        motion_command_source_ = "navigation";
        motion_command_prefix_ = x30_udp_protocol::kNavigationCommandPrefix;
    }
}

DeepRoboticsX30::~DeepRoboticsX30()
{
    heartbeat_running_ = false;
    velocity_watchdog_running_ = false;

    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    if (velocity_watchdog_thread_.joinable()) {
        velocity_watchdog_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

int32_t DeepRoboticsX30::initRobotHardware()
{
    if (is_initialized_) {
        return CMD_SUCCESS;
    }

    if (max_vx_ <= 0.0 || max_vy_ <= 0.0 || max_omega_ <= 0.0) {
        std::cerr << "[DeepRoboticsX30] max_vx, max_vy, and max_omega "
                  << "must all be greater than 0." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (velocity_watchdog_hz_ <= 0 || velocity_command_timeout_ms_ <= 0 ||
        velocity_zero_hold_ms_ < 0) {
        std::cerr << "[DeepRoboticsX30] Invalid velocity watchdog configuration."
                  << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[DeepRoboticsX30] Failed to create UDP socket: "
                  << std::strerror(errno) << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (local_port_ > 0) {
        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(static_cast<uint16_t>(local_port_));

        if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
            std::cerr << "[DeepRoboticsX30] Failed to bind UDP local port "
                      << local_port_ << ": " << std::strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return ERROR_ROBOT_HARDWARE_INIT;
        }
    }

    motion_addr_ = makeUdpAddress(motion_ip_, motion_port_);
    perception_addr_ = makeUdpAddress(perception_ip_, perception_port_);
    navigation_velocity_addr_ =
        makeUdpAddress(navigation_velocity_ip_, navigation_velocity_port_);

    is_initialized_ = true;

    if (sendMotionCommand(motionCode(X30_HEARTBEAT_SUFFIX)) != CMD_SUCCESS ||
        sendMotionCommand(motionCode(X30_CONFIRM_CONNECTION_SUFFIX)) != CMD_SUCCESS) {
        is_initialized_ = false;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (configure_non_manual_mode_) {
        sendMotionCommand(motionCode(X30_NON_MANUAL_MODE_SUFFIX));
    }

    if (configure_navigation_velocity_source_) {
        sendPerceptionCommand(X30_VELOCITY_SOURCE, 2);
    }

    int32_t heartbeat_ret = startHeartbeat();
    if (heartbeat_ret != CMD_SUCCESS) {
        is_initialized_ = false;
        return heartbeat_ret;
    }

    int32_t watchdog_ret = startVelocityWatchdog();
    if (watchdog_ret != CMD_SUCCESS) {
        heartbeat_running_ = false;
        if (heartbeat_thread_.joinable()) {
            heartbeat_thread_.join();
        }
        is_initialized_ = false;
        return watchdog_ret;
    }

    std::cout << "[DeepRoboticsX30] UDP initialized. motion="
              << motion_ip_ << ":" << motion_port_
              << ", perception=" << perception_ip_ << ":" << perception_port_
              << ", navigation_velocity=" << navigation_velocity_ip_
              << ":" << navigation_velocity_port_
              << ", motion_command_source=" << motion_command_source_
              << ", velocity_backend=" << velocityBackendName()
              << std::endl;

    if (velocity_backend_ == VelocityBackend::UDP_AXIS) {
        std::cout << "[DeepRoboticsX30] Note: udp_axis sends joystick-axis commands. "
                  << "The configured max velocities are mapping limits, not a guarantee "
                  << "that RL-state motion tracks SI-unit velocity commands."
                  << std::endl;
    } else if (velocity_backend_ == VelocityBackend::UDP_PHYSICAL) {
        std::cout << "[DeepRoboticsX30] Note: udp_physical matches the factory "
                  << "/cmd_vel_corrected output and bypasses the perception safety layer."
                  << std::endl;
    } else {
        std::cout << "[DeepRoboticsX30] Note: udp_navigation sends factory 0x150 "
                  << "navigation velocity packets to the perception host."
                  << std::endl;
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::writeRobotVelocityCommand(RobotVelocityCommand &cmd)
{
    if (!is_initialized_) {
        std::cerr << "[DeepRoboticsX30] Error: robot not initialized." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) ||
        !std::isfinite(cmd.omega)) {
        std::cerr << "[DeepRoboticsX30] Velocity command contains NaN or infinity."
                  << std::endl;
        return ERROR_ROBOT_HARDWARE_MOVE;
    }

    RobotVelocityCommand safe_cmd{
        std::clamp(cmd.vx, -max_vx_, max_vx_),
        std::clamp(cmd.vy, -max_vy_, max_vy_),
        std::clamp(cmd.omega, -max_omega_, max_omega_),
    };

    std::lock_guard<std::mutex> lock(velocity_command_mutex_);
    int32_t ret = sendVelocityCommand(safe_cmd);
    if (ret != CMD_SUCCESS) {
        return ret;
    }

    const bool is_nonzero =
        std::abs(safe_cmd.vx) > X30_ZERO_VELOCITY_EPSILON ||
        std::abs(safe_cmd.vy) > X30_ZERO_VELOCITY_EPSILON ||
        std::abs(safe_cmd.omega) > X30_ZERO_VELOCITY_EPSILON;
    velocity_command_active_ = is_nonzero;
    last_velocity_command_time_ = std::chrono::steady_clock::now();

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::writeActionCommand(std::string action)
{
    if (!is_initialized_) {
        std::cerr << "[DeepRoboticsX30] Error: robot not initialized." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    if (action == ACTION_STOP_MOVE) {
        return sendStopCommand();
    }

    if (action == ACTION_START_STOP_MOTION) {
        return sendMotionCommand(motionCode(X30_START_STOP_MOTION_SUFFIX));
    }

    if (action == ACTION_GAIT_MOUNTAIN) {
        return sendGaitCommand(
            x30_udp_protocol::kGaitMountainSuffix, "mountain");
    }

    if (action == ACTION_GAIT_WALK) {
        return sendGaitCommand(x30_udp_protocol::kGaitWalkSuffix, "walk");
    }

    if (action == ACTION_GAIT_L_WALK) {
        return sendGaitCommand(x30_udp_protocol::kGaitLWalkSuffix, "L-walk");
    }

    if (action == ACTION_STAND_UP || action == ACTION_LIE_DOWN) {
        std::cerr << "[DeepRoboticsX30] Warning: " << action
                  << " maps to an X30 toggle command. Verify robot state before using it."
                  << std::endl;
        return sendMotionCommand(motionCode(X30_STAND_UP_LIE_DOWN_SUFFIX));
    }

    std::cerr << "[DeepRoboticsX30] Unsupported action: " << action << std::endl;
    return ERROR_ROBOT_HARDWARE_ACTION_FAILED;
}

int32_t DeepRoboticsX30::sendVelocityCommand(const RobotVelocityCommand &cmd)
{
    switch (velocity_backend_) {
        case VelocityBackend::UDP_AXIS:
            return sendAxisVelocityCommand(cmd);
        case VelocityBackend::UDP_PHYSICAL:
            return sendPhysicalVelocityCommand(cmd);
        case VelocityBackend::UDP_NAVIGATION:
            return sendNavigationVelocityCommand(cmd);
    }

    return ERROR_ROBOT_HARDWARE_MOVE;
}

int32_t DeepRoboticsX30::sendAxisVelocityCommand(const RobotVelocityCommand &cmd)
{
    const int32_t vx_axis =
        velocityToAxis(cmd.vx, max_vx_, X30_VX_DEADZONE, false);
    const int32_t vy_axis =
        velocityToAxis(cmd.vy, max_vy_, X30_VY_DEADZONE, invert_vy_axis_);
    const int32_t yaw_axis = yawRateToAxis(cmd.omega);

    if (sendAxisCommand(motionCode(X30_AXIS_VX_SUFFIX), vx_axis) != CMD_SUCCESS ||
        sendAxisCommand(motionCode(X30_AXIS_VY_SUFFIX), vy_axis) != CMD_SUCCESS ||
        sendAxisCommand(motionCode(X30_AXIS_YAW_SUFFIX), yaw_axis) != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_MOVE;
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::sendPhysicalVelocityCommand(
    const RobotVelocityCommand &cmd)
{
    const int32_t vx_milli =
        x30_udp_protocol::physicalVelocityToMilli(cmd.vx);
    const int32_t vy_milli =
        x30_udp_protocol::physicalVelocityToMilli(cmd.vy);
    const int32_t omega_milli =
        x30_udp_protocol::physicalVelocityToMilli(cmd.omega);

    if (sendMotionCommand(x30_udp_protocol::kPhysicalVxCode, vx_milli) != CMD_SUCCESS ||
        sendMotionCommand(x30_udp_protocol::kPhysicalVyCode, vy_milli) != CMD_SUCCESS ||
        sendMotionCommand(x30_udp_protocol::kPhysicalYawCode, omega_milli) != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_MOVE;
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::sendNavigationVelocityCommand(
    const RobotVelocityCommand &cmd)
{
    const auto packet = x30_udp_protocol::makeNavigationVelocityPacket(
        cmd.vx, cmd.vy, cmd.omega);

    int32_t ret = sendDatagram(
        navigation_velocity_addr_,
        packet.data(),
        packet.size(),
        x30_udp_protocol::kNavigationVelocityCode);
    return ret == CMD_SUCCESS ? CMD_SUCCESS : ERROR_ROBOT_HARDWARE_MOVE;
}

int32_t DeepRoboticsX30::sendDatagram(const sockaddr_in &target_addr,
                                      const void *data,
                                      std::size_t size,
                                      uint32_t code_for_log)
{
    if (socket_fd_ < 0) {
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    const ssize_t sent = sendto(
        socket_fd_,
        data,
        size,
        0,
        reinterpret_cast<const sockaddr *>(&target_addr),
        sizeof(target_addr)
    );

    if (sent != static_cast<ssize_t>(size)) {
        std::cerr << "[DeepRoboticsX30] sendto failed for command 0x"
                  << std::hex << code_for_log << std::dec
                  << ": " << std::strerror(errno) << std::endl;
        return CMD_ERROR;
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::sendSimpleCommand(const sockaddr_in &target_addr, uint32_t code, int32_t value)
{
    const auto packet = x30_udp_protocol::makeSimplePacket(code, value);
    return sendDatagram(target_addr, packet.data(), packet.size(), code);
}

int32_t DeepRoboticsX30::sendMotionCommand(uint32_t code, int32_t value)
{
    return sendSimpleCommand(motion_addr_, code, value);
}

int32_t DeepRoboticsX30::sendPerceptionCommand(uint32_t code, int32_t value)
{
    return sendSimpleCommand(perception_addr_, code, value);
}

int32_t DeepRoboticsX30::sendAxisCommand(uint32_t code, int32_t axis_value)
{
    axis_value = std::clamp(axis_value, -X30_AXIS_MAX, X30_AXIS_MAX);
    return sendMotionCommand(code, axis_value);
}

int32_t DeepRoboticsX30::sendStopCommand()
{
    RobotVelocityCommand stop_cmd{0.0, 0.0, 0.0};

    for (int i = 0; i < 20; ++i) {
        int32_t ret = writeRobotVelocityCommand(stop_cmd);
        if (ret != CMD_SUCCESS) {
            return ret;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::sendGaitCommand(uint32_t gait_suffix,
                                         const std::string &gait_name)
{
    const RobotVelocityCommand stop_cmd{0.0, 0.0, 0.0};
    std::lock_guard<std::mutex> lock(velocity_command_mutex_);

    // Block concurrent velocity writes and clear stale motion before changing gait.
    for (int i = 0; i < 20; ++i) {
        if (sendVelocityCommand(stop_cmd) != CMD_SUCCESS) {
            std::cerr << "[DeepRoboticsX30] Failed to send zero velocity before "
                      << gait_name << " gait command." << std::endl;
            velocity_command_active_ = false;
            return ERROR_ROBOT_HARDWARE_ACTION_FAILED;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    velocity_command_active_ = false;
    last_velocity_command_time_ = std::chrono::steady_clock::now();

    const uint32_t command = motionCode(gait_suffix);
    std::cout << "[DeepRoboticsX30] Sending " << gait_name
              << " gait command 0x" << std::hex << command << std::dec
              << ". UDP send success does not confirm robot state; verify "
              << "/robot_gait_state." << std::endl;

    if (sendMotionCommand(command) != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_ACTION_FAILED;
    }

    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::startHeartbeat()
{
    if (heartbeat_hz_ <= 0) {
        std::cerr << "[DeepRoboticsX30] heartbeat_hz must be greater than 0." << std::endl;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    heartbeat_running_ = true;
    heartbeat_thread_ = std::thread(&DeepRoboticsX30::heartbeatLoop, this);

    return CMD_SUCCESS;
}

void DeepRoboticsX30::heartbeatLoop()
{
    auto period = std::chrono::milliseconds(std::max(1, 1000 / heartbeat_hz_));

    while (heartbeat_running_) {
        sendMotionCommand(motionCode(X30_HEARTBEAT_SUFFIX));
        std::this_thread::sleep_for(period);
    }
}

int32_t DeepRoboticsX30::startVelocityWatchdog()
{
    velocity_watchdog_running_ = true;
    velocity_watchdog_thread_ =
        std::thread(&DeepRoboticsX30::velocityWatchdogLoop, this);
    return CMD_SUCCESS;
}

void DeepRoboticsX30::velocityWatchdogLoop()
{
    const auto period =
        std::chrono::milliseconds(std::max(1, 1000 / velocity_watchdog_hz_));
    const auto command_timeout =
        std::chrono::milliseconds(velocity_command_timeout_ms_);
    const auto zero_hold = std::chrono::milliseconds(velocity_zero_hold_ms_);
    const RobotVelocityCommand stop_cmd{0.0, 0.0, 0.0};

    while (velocity_watchdog_running_) {
        std::this_thread::sleep_for(period);

        std::lock_guard<std::mutex> lock(velocity_command_mutex_);
        if (!velocity_command_active_) {
            continue;
        }

        const auto elapsed =
            std::chrono::steady_clock::now() - last_velocity_command_time_;
        if (elapsed < command_timeout) {
            continue;
        }

        if (elapsed > command_timeout + zero_hold) {
            velocity_command_active_ = false;
            continue;
        }

        if (sendVelocityCommand(stop_cmd) != CMD_SUCCESS) {
            std::cerr << "[DeepRoboticsX30] Velocity watchdog failed to send stop."
                      << std::endl;
            velocity_command_active_ = false;
        }
    }
}

uint32_t DeepRoboticsX30::motionCode(uint32_t suffix) const
{
    return x30_udp_protocol::makeMotionCommandCode(
        motion_command_prefix_, suffix);
}

std::string DeepRoboticsX30::velocityBackendName() const
{
    switch (velocity_backend_) {
        case VelocityBackend::UDP_AXIS:
            return "udp_axis";
        case VelocityBackend::UDP_PHYSICAL:
            return "udp_physical";
        case VelocityBackend::UDP_NAVIGATION:
            return "udp_navigation";
    }

    return "unknown";
}

int32_t DeepRoboticsX30::velocityToAxis(double velocity, double max_velocity, int32_t deadzone, bool invert) const
{
    if (max_velocity <= 0.0 || std::abs(velocity) < 1e-6) {
        return 0;
    }

    double ratio = std::clamp(std::abs(velocity) / max_velocity, 0.0, 1.0);

    int sign = velocity >= 0.0 ? 1 : -1;
    if (invert) {
        sign *= -1;
    }

    int32_t axis_abs = deadzone + static_cast<int32_t>(
        std::lround(ratio * static_cast<double>(X30_AXIS_MAX - deadzone))
    );

    axis_abs = std::clamp(axis_abs, deadzone, X30_AXIS_MAX);

    return sign * axis_abs;
}

int32_t DeepRoboticsX30::yawRateToAxis(double yaw_rate) const
{
    return velocityToAxis(yaw_rate, max_omega_, X30_YAW_DEADZONE, invert_omega_axis_);
}
