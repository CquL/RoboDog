// X30 HAL 实现：把通用接口映射为厂家 UDP 数据包，并负责 heartbeat/watchdog；
// 导航与地形规划不属于本层职责。
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

// 运动主机动作后缀。motionCode() 会按配置补上 remote (0x21) 或
// navigation (0x31) 来源字节。
constexpr uint32_t X30_HEARTBEAT_SUFFIX = 0x00040001;
constexpr uint32_t X30_CONFIRM_CONNECTION_SUFFIX = 0x00020001;
constexpr uint32_t X30_NON_MANUAL_MODE_SUFFIX = 0x00010C03;
constexpr uint32_t X30_VELOCITY_SOURCE = 0x3101EE03;
constexpr uint32_t X30_STAND_UP_LIE_DOWN_SUFFIX = 0x00010202;
constexpr uint32_t X30_START_STOP_MOTION_SUFFIX = 0x00010201;

// Axis 命令对应厂家的虚拟摇杆通道，与 x30_udp_protocol.h 中的
// physical、navigation 速度命令分开定义。
constexpr uint32_t X30_AXIS_VX_SUFFIX = 0x00010130;
constexpr uint32_t X30_AXIS_VY_SUFFIX = 0x00010131;
constexpr uint32_t X30_AXIS_YAW_SUFFIX = 0x00010135;

// Axis 模式模拟厂家控制器的整数摇杆范围。neutral/deadzone 是协议属性，
// 不是物理速度单位。
constexpr int32_t X30_AXIS_MAX = 32767;
constexpr int32_t X30_VX_DEADZONE = 6553;
constexpr int32_t X30_VY_DEADZONE = 24576;
constexpr int32_t X30_YAW_DEADZONE = 28212;
constexpr double X30_ZERO_VELOCITY_EPSILON = 1e-9;

// 可选 YAML 键缺失时保留类内默认值，使精简 X30 配置仍可运行，
// 同时允许覆盖每个运行参数。
template <typename T>
T readYaml(const YAML::Node &node, const std::string &key, const T &default_value)
{
    if (node && node[key]) {
        return node[key].as<T>();
    }
    return default_value;
}

// 初始化时一次性构造 sockaddr，避免控制循环反复解析 IP 文本。
sockaddr_in makeUdpAddress(const std::string &ip, int port)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    return addr;
}

} // namespace

// 构造阶段只把配置解析为协议选项；若在此打开 socket，
// RobotFactory 分配对象时就会产生网络副作用。
DeepRoboticsX30::DeepRoboticsX30(YAML::Node config) : RobotHardwareInterface(config)
{
    // motion、perception、navigation 端点对应不同厂家进程，
    // 即使多个端点位于同一台物理主机也分别配置。
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

    // Axis 反向在封包前修正厂家摇杆符号约定，
    // 不改变通用机身坐标 API。
    invert_vy_axis_ = readYaml<bool>(config, "invert_vy_axis", invert_vy_axis_);
    invert_omega_axis_ = readYaml<bool>(config, "invert_omega_axis", invert_omega_axis_);

    configure_non_manual_mode_ =
        readYaml<bool>(config, "configure_non_manual_mode", configure_non_manual_mode_);

    configure_navigation_velocity_source_ =
        readYaml<bool>(config, "configure_navigation_velocity_source", configure_navigation_velocity_source_);

    // 每次选择一条完整速度链；同一命令流混用数据包格式，
    // 会导致厂家控制器对数值作出不一致解释。
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

    // 动作命令码独立使用配置的来源前缀，因为厂家将步态选择与
    // 速度路由视为两项独立设置。
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

// 先停止后台循环，再关闭其与前台调用共享的 UDP socket。
// join 可防止 heartbeat 或 watchdog 在 fd 关闭后继续发送。
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

// 初始化建立传输与厂家会话状态，再启动 heartbeat 和陈旧命令 watchdog；
// 此处不会发送非零速度。
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

    // 一个未 connect 的 datagram socket 即可，因为 sendto()
    // 会为每个 motion/perception/navigation 数据包指定目标地址。
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

    // 校验后缓存三个目标地址，使实时路径只需序列化并发送已解析的数据包。
    motion_addr_ = makeUdpAddress(motion_ip_, motion_port_);
    perception_addr_ = makeUdpAddress(perception_ip_, perception_port_);
    navigation_velocity_addr_ =
        makeUdpAddress(navigation_velocity_ip_, navigation_velocity_port_);

    is_initialized_ = true;

    // 后台线程启动前，先用初始 heartbeat 和连接确认命令验证
    // 本地 socket 能发送所需厂家帧。
    if (sendMotionCommand(motionCode(X30_HEARTBEAT_SUFFIX)) != CMD_SUCCESS ||
        sendMotionCommand(motionCode(X30_CONFIRM_CONNECTION_SUFFIX)) != CMD_SUCCESS) {
        is_initialized_ = false;
        return ERROR_ROBOT_HARDWARE_INIT;
    }

    // 修改厂家控制模式或速度源属于系统级副作用，不是传输初始化，
    // 因此这些写操作必须由配置显式启用。
    if (configure_non_manual_mode_) {
        sendMotionCommand(motionCode(X30_NON_MANUAL_MODE_SUFFIX));
    }

    if (configure_navigation_velocity_source_) {
        sendPerceptionCommand(X30_VELOCITY_SOURCE, 2);
    }

    // heartbeat 先启动；若 watchdog 启动失败，则回收 heartbeat，
    // 确保初始化失败后没有后台发送线程残留。
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

// 这是唯一公开的速度写入路径：拒绝非有限值、应用当前限幅、
// 发送一条完整后端命令，成功后才将命令流标记为由 watchdog 监管。
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

    std::lock_guard<std::mutex> lock(velocity_command_mutex_);

    // 普通楼梯步态采用项目全局限幅与文档步态限幅的交集；
    // 其他步态继续使用项目配置限幅。
    const double active_max_vx = ordinary_stairs_limits_active_
        ? std::min(max_vx_, x30_udp_protocol::kGaitStairsMaxVx)
        : max_vx_;
    const double active_max_vy = ordinary_stairs_limits_active_
        ? std::min(max_vy_, x30_udp_protocol::kGaitStairsMaxVy)
        : max_vy_;
    const double active_max_omega = ordinary_stairs_limits_active_
        ? std::min(max_omega_, x30_udp_protocol::kGaitStairsMaxOmega)
        : max_omega_;
    RobotVelocityCommand safe_cmd{
        std::clamp(cmd.vx, -active_max_vx, active_max_vx),
        std::clamp(cmd.vy, -active_max_vy, active_max_vy),
        std::clamp(cmd.omega, -active_max_omega, active_max_omega),
    };

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

// 将稳定的字符串动作词汇映射到厂家命令。切换类动作保留明确告警，
// 因为 UDP 发送成功无法说明机器人初始姿态或最终状态。
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

    if (action == ACTION_GAIT_STAIRS) {
        return sendGaitCommand(
            x30_udp_protocol::kGaitStairsSuffix, "ordinary stairs");
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

// 将后端选择集中在一个 switch，避免公开调用方为同一命令
// 误发 Axis 与 SI 单位两种数据包格式。
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

// Axis 模式把各 SI 单位分量映射为有符号虚拟摇杆通道，
// 再向运动主机分别发送三个通道命令。
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

// Physical 模式复现厂家修正速度发送方式：各分量乘以 1000，
// 并作为独立帧直接发往运动主机。
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

// Navigation 模式保留 SI 单位 double，以一个完整 twist 数据包发往感知主机，
// 由选定的厂家导航链路消费。
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

// UDP 没有交付确认；此处成功只表示内核接收了一个完整 datagram。
// 实际执行结果必须由机器人状态 Topic 确认。
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

// 简单命令共用 12 字节协议帧，仅目标端点、命令码和有符号值不同。
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

// 连续发送 200 ms 零速度可覆盖瞬时丢包，
// 并让下游命令消费者多次收到停止请求。
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

    // 切换步态前阻止并发速度写入，并清除残留运动命令。
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

    // 步态请求只发送一次目标动作；本 HAL 不插入中间步态，
    // 也不自动恢复先前步态。
    const uint32_t command = motionCode(gait_suffix);
    std::cout << "[DeepRoboticsX30] Sending " << gait_name
              << " gait command 0x" << std::hex << command << std::dec
              << ". UDP send success does not confirm robot state; verify "
              << "/robot_gait_state." << std::endl;

    if (sendMotionCommand(command) != CMD_SUCCESS) {
        return ERROR_ROBOT_HARDWARE_ACTION_FAILED;
    }

    // 本地记录普通楼梯步态限幅。UDP 发送成功后，
    // 安全调用方仍须取得外部步态反馈才能开始运动。
    ordinary_stairs_limits_active_ =
        gait_suffix == x30_udp_protocol::kGaitStairsSuffix;
    if (ordinary_stairs_limits_active_) {
        std::cout
            << "[DeepRoboticsX30] Ordinary-stairs velocity limits active: "
            << "|vx|<=" << x30_udp_protocol::kGaitStairsMaxVx
            << " m/s, |vy|<=" << x30_udp_protocol::kGaitStairsMaxVy
            << " m/s, |omega|<="
            << x30_udp_protocol::kGaitStairsMaxOmega << " rad/s."
            << std::endl;
    }

    return CMD_SUCCESS;
}

// heartbeat 与 watchdog 独立运行：前者维持厂家会话，
// 后者监管上层速度命令的新鲜度。
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

        // 与前台写入和步态切换共用同一 mutex，
        // 使每次超时判定及其零速度发送相对这些操作保持原子性。
        std::lock_guard<std::mutex> lock(velocity_command_mutex_);
        if (!velocity_command_active_) {
            continue;
        }

        const auto elapsed =
            std::chrono::steady_clock::now() - last_velocity_command_time_;
        if (elapsed < command_timeout) {
            continue;
        }

        // zero_hold 期间 watchdog 按配置频率刷新零速度；
        // 结束后将命令流视为非活动，避免无限发送。
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

// 动作后缀与来源无关；此 helper 在序列化前附加配置的
// remote/navigation 高字节。
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

// Axis 零值使用特殊 neutral 值；非零命令从已恢复的通道 deadzone 起，
// 线性缩放到控制器有符号最大值。
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
