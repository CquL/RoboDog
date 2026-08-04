// X30 统一现场测试 CLI。步态命令只发送目标动作并保持该步态；
// 运动模式持续发送限幅速度，结束时发送零速度。
// ROS1 反馈校验可在构建时选择启用。
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "deep_robotics/x30_udp_protocol.h"
#include "robot_factory.h"

#ifdef X30_TEST_WITH_ROS1
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include <mutex>
#endif

namespace {

// 所有后端按相同固定频率发送。步态反馈使用独立超时，
// 因为 UDP 发送完成不能证明动作已执行。
constexpr int kCommandHz = 50;
constexpr int kZeroHoldMs = 500;
constexpr int kGaitFeedbackTimeoutMs = 5000;
constexpr char kMoveConfirmation[] = "CONFIRM_MOVE";

// 参数解析同时支持 "test command" 与 "test config.yaml command"，
// 并将命令专用参数处理隔离在 main() 之外。
struct CommandLine {
    std::string config_path = "../config.yaml";
    std::string command;
    int argument_index = 0;
};

// 步态条目关联 CLI 名称、通用 HAL 动作，以及用于可选执行确认的
// 厂家 ROS1 状态值。
struct GaitCommand {
    const char *name;
    const std::string *action;
    int feedback_state;
};

// 仅对已知命令推断配置路径，避免把命令拼写错误当作 YAML 文件名。
bool isTopLevelCommand(const std::string &value)
{
    return value == "help" || value == "--help" || value == "list" ||
           value == "status" || value == "zero" || value == "stop" ||
           value == "gait" || value == "move" || value == "walk" ||
           value == "l_walk" || value == "mountain" || value == "stairs";
}

// 定位命令及首个命令参数，同时保留默认配置路径，便于目标机使用。
bool parseCommandLine(int argc, char **argv, CommandLine &parsed)
{
    if (argc < 2) {
        return false;
    }

    if (isTopLevelCommand(argv[1])) {
        parsed.command = argv[1];
        parsed.argument_index = 2;
        return true;
    }

    if (argc < 3 || !isTopLevelCommand(argv[2])) {
        return false;
    }

    parsed.config_path = argv[1];
    parsed.command = argv[2];
    parsed.argument_index = 3;
    return true;
}

// Usage 文本同时定义行为约定：步态命令保持目标步态；
// move 始终以连续零速度结束，且不会隐式切换步态。
void printUsage(const char *program)
{
    std::cout
        << "X30 unified control test\n\n"
        << "Usage:\n"
        << "  " << program << " [config.yaml] status\n"
        << "  " << program << " [config.yaml] zero\n"
        << "  " << program << " [config.yaml] gait "
        << "walk|l_walk|mountain|stairs\n"
        << "  " << program << " [config.yaml] "
        << "walk|l_walk|mountain|stairs\n"
        << "  " << program << " [config.yaml] move "
        << "<vx> <vy> <omega> <duration_s> " << kMoveConfirmation << "\n\n"
        << "Behavior:\n"
        << "  status  Read state feedback when built with ROS1; send no UDP.\n"
        << "  zero    Send zero velocity and leave the current gait unchanged.\n"
        << "  gait    Send exactly one gait action; no intermediate or recovery gait.\n"
        << "  move    Send the requested velocity at " << kCommandHz
        << " Hz for exactly the requested duration, then send zero velocity.\n\n"
        << "Examples:\n"
        << "  " << program << " ../config.yaml stairs\n"
        << "  " << program << " ../config.yaml move 0.30 0 0 5 "
        << kMoveConfirmation << "\n"
        << "  " << program << " ../config.yaml move 0 0 0.30 2 "
        << kMoveConfirmation << "\n";
}

// strtod 后检查完整输入和有限性，防止 NaN、infinity
// 或部分解析值进入实机命令。
bool parseFiniteDouble(const char *text, double &value)
{
    if (text == nullptr) {
        return false;
    }

    char *end = nullptr;
    value = std::strtod(text, &end);
    return end != text && end != nullptr && *end == '\0' &&
           std::isfinite(value);
}

// 状态名仅用于诊断。值 7、8 是可观测厂家状态，
// 但普通楼梯测试不把它们暴露为命令。
const char *gaitName(int gait)
{
    switch (gait) {
        case 0:
            return "walk";
        case 6:
            return "stairs";
        case 7:
            return "stairs_accumulated";
        case 8:
            return "stairs_45";
        case 32:
            return "l_walk";
        case 33:
            return "mountain";
        case 34:
            return "silent";
        default:
            return "other";
    }
}

// 将 CLI 到 action 的映射集中在一个表式函数中，
// 使步态别名和显式 "gait" 子命令行为一致。
bool lookupGait(const std::string &name, GaitCommand &gait)
{
    if (name == "walk") {
        gait = {"walk", &ACTION_GAIT_WALK, 0};
        return true;
    }
    if (name == "l_walk") {
        gait = {"l_walk", &ACTION_GAIT_L_WALK, 32};
        return true;
    }
    if (name == "mountain") {
        gait = {"mountain", &ACTION_GAIT_MOUNTAIN, 33};
        return true;
    }
    if (name == "stairs") {
        gait = {"stairs", &ACTION_GAIT_STAIRS, 6};
        return true;
    }
    return false;
}

// 在 RobotFactory 分配其他后端前拒绝非 X30 配置。
YAML::Node loadX30Config(const std::string &path)
{
    YAML::Node config = YAML::LoadFile(path);
    if (!config["robot_model"] ||
        config["robot_model"].as<std::string>() != "deep_robotics_x30") {
        throw std::runtime_error(
            "config robot_model must be deep_robotics_x30");
    }
    return config;
}

// 停止命令采用连续发送而非单次发送，因为 X30 命令链使用 UDP，
// 且下游消费者要求持续速度源。
bool sendZeroFor(const std::shared_ptr<RobotHardwareInterface> &robot,
                 int duration_ms)
{
    RobotVelocityCommand zero{0.0, 0.0, 0.0};
    const auto period = std::chrono::milliseconds(1000 / kCommandHz);
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(duration_ms);

    bool ok = true;
    do {
        if (robot->writeRobotVelocityCommand(zero) != CMD_SUCCESS) {
            ok = false;
        }
        std::this_thread::sleep_for(period);
    } while (std::chrono::steady_clock::now() < end);
    return ok;
}

#ifdef X30_TEST_WITH_ROS1

// 命令循环运行时，ROS callback 由 spinOnce() 执行。每个 tracker
// 保存单调接收时间，使调用方能区分“从未收到”和合法零值状态。
struct IntSample {
    int value = 0;
    std::chrono::steady_clock::time_point received_at{};
};

struct OdomSample {
    double x = 0.0;
    double y = 0.0;
    std::chrono::steady_clock::time_point received_at{};
};

struct VelocitySample {
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    std::chrono::steady_clock::time_point received_at{};
};

// 各 tracker 类在 mutex 保护下复制最新样本，
// 将 ROS callback 所有权与状态、运动报告解耦。
class IntTracker {
public:
    void callback(const std_msgs::Int32::ConstPtr &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.value = message->data;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(IntSample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_sample_) {
            return false;
        }
        sample = sample_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    IntSample sample_;
    bool has_sample_ = false;
};

// /control_mode 类型为 UInt8，其他状态 Topic 为 Int32，
// 因此使用独立 callback，但复用统一的 IntSample 表示。
class UInt8Tracker {
public:
    void callback(const std_msgs::UInt8::ConstPtr &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.value = message->data;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(IntSample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_sample_) {
            return false;
        }
        sample = sample_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    IntSample sample_;
    bool has_sample_ = false;
};

// Odometry 只保存平面位置，因为现场测试报告水平位移，
// 不评估完整定位结果。
class OdomTracker {
public:
    void callback(const nav_msgs::Odometry::ConstPtr &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.x = message->pose.pose.position.x;
        sample_.y = message->pose.pose.position.y;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(OdomSample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_sample_) {
            return false;
        }
        sample = sample_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    OdomSample sample_;
    bool has_sample_ = false;
};

// 速度反馈保留与通用 HAL 命令相同的 vx/vy/omega 分量，
// 便于直接比较最终诊断结果。
class VelocityTracker {
public:
    void callback(const geometry_msgs::Twist::ConstPtr &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.vx = message->linear.x;
        sample_.vy = message->linear.y;
        sample_.omega = message->angular.z;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(VelocitySample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_sample_) {
            return false;
        }
        sample = sample_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    VelocitySample sample_;
    bool has_sample_ = false;
};

// status 等待所有必需状态 Topic 各收到一个样本，
// 使输出代表完整可用性检查，而不是局部证据。
bool waitForRobotState(IntTracker &basic_tracker,
                       IntTracker &gait_tracker,
                       UInt8Tracker &control_tracker,
                       IntTracker &terrain_tracker,
                       IntSample &basic,
                       IntSample &gait,
                       IntSample &control,
                       IntSample &terrain,
                       int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        if (basic_tracker.latest(basic) && gait_tracker.latest(gait) &&
            control_tracker.latest(control) &&
            terrain_tracker.latest(terrain)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// 只有 ROS1 厂家状态等于目标值才确认步态切换成功；
// 仅 UDP 发送成功不足以确认。
bool waitForGait(IntTracker &tracker, int expected_state, int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        IntSample sample;
        if (tracker.latest(sample) && sample.value == expected_state) {
            std::cout << "[robot_test_x30] Verified gait_state="
                      << sample.value << " (" << gaitName(sample.value)
                      << ")." << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// 现场位移仅用于诊断，不作为闭环停止条件。
double planarDistance(const OdomSample &start, const OdomSample &end)
{
    return std::hypot(end.x - start.x, end.y - start.y);
}

#endif

// status 明确为只读，不构造机器人 HAL 对象，
// 因此不会初始化 UDP 或改变控制模式。
int runStatus()
{
#ifdef X30_TEST_WITH_ROS1
    ros::NodeHandle node;
    IntTracker basic_tracker;
    IntTracker gait_tracker;
    UInt8Tracker control_tracker;
    IntTracker terrain_tracker;

    const auto basic_sub = node.subscribe<std_msgs::Int32>(
        "/robot_basic_state", 20, &IntTracker::callback, &basic_tracker);
    const auto gait_sub = node.subscribe<std_msgs::Int32>(
        "/robot_gait_state", 20, &IntTracker::callback, &gait_tracker);
    const auto control_sub = node.subscribe<std_msgs::UInt8>(
        "/control_mode", 20, &UInt8Tracker::callback, &control_tracker);
    const auto terrain_sub = node.subscribe<std_msgs::Int32>(
        "/height_map_mode_state", 20, &IntTracker::callback, &terrain_tracker);
    (void)basic_sub;
    (void)gait_sub;
    (void)control_sub;
    (void)terrain_sub;

    IntSample basic;
    IntSample gait;
    IntSample control;
    IntSample terrain;
    if (!waitForRobotState(
            basic_tracker,
            gait_tracker,
            control_tracker,
            terrain_tracker,
            basic,
            gait,
            control,
            terrain,
            5000)) {
        std::cerr << "[robot_test_x30] Timed out waiting for ROS1 state."
                  << std::endl;
        return 1;
    }

    std::cout << "[robot_test_x30] basic_state=" << basic.value
              << ", gait_state=" << gait.value << " ("
              << gaitName(gait.value) << ")"
              << ", control_mode=" << control.value
              << ", height_map_mode_state=" << terrain.value << "."
              << std::endl;
    std::cout << "[robot_test_x30] Status sent no UDP command." << std::endl;
    return 0;
#else
    std::cerr
        << "[robot_test_x30] status requires BUILD_X30_ROS1_TEST=ON."
        << std::endl;
    return 1;
#endif
}

// 只发送一次目标步态动作；若具备 ROS1 支持，
// 则验证厂家步态结果，不插入任何恢复动作。
int runGait(const YAML::Node &config, const GaitCommand &gait)
{
#ifdef X30_TEST_WITH_ROS1
    ros::NodeHandle node;
    IntTracker gait_tracker;
    const auto gait_sub = node.subscribe<std_msgs::Int32>(
        "/robot_gait_state", 20, &IntTracker::callback, &gait_tracker);
    (void)gait_sub;
#endif

    const auto robot = RobotFactory::RobotAllocate(config);
    const int32_t init_result = robot->initRobotHardware();
    if (init_result != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30] Robot init failed, ret="
                  << init_result << "." << std::endl;
        return 1;
    }

    std::cout << "[robot_test_x30] Sending gait " << gait.name
              << " directly. No intermediate or recovery gait is sent."
              << std::endl;
    const int32_t action_result = robot->writeActionCommand(*gait.action);
    if (action_result != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30] Gait action failed, ret="
                  << action_result << "." << std::endl;
        return 1;
    }

#ifdef X30_TEST_WITH_ROS1
    if (!waitForGait(
            gait_tracker, gait.feedback_state, kGaitFeedbackTimeoutMs)) {
        std::cerr << "[robot_test_x30] Gait command was sent, but "
                  << "gait_state=" << gait.feedback_state
                  << " was not observed." << std::endl;
        return 1;
    }
#else
    std::cout << "[robot_test_x30] Gait command sent. This build has no ROS1 "
              << "feedback verification." << std::endl;
#endif
    return 0;
}

// 运动按请求时长开环执行。配置限幅和显式确认 token 是前置条件；
// 命令循环结束后始终尝试连续发送零速度，包括发送失败情形。
int runMove(const YAML::Node &config,
            double vx,
            double vy,
            double omega,
            double duration_s)
{
    const double max_vx = config["max_vx"]
        ? config["max_vx"].as<double>()
        : 0.0;
    const double max_vy = config["max_vy"]
        ? config["max_vy"].as<double>()
        : 0.0;
    const double max_omega = config["max_omega"]
        ? config["max_omega"].as<double>()
        : 0.0;

    if (duration_s <= 0.0 ||
        std::abs(vx) > max_vx ||
        std::abs(vy) > max_vy ||
        std::abs(omega) > max_omega) {
        std::cerr << "[robot_test_x30] Invalid move. Config limits are "
                  << "|vx|<=" << max_vx << ", |vy|<=" << max_vy
                  << ", |omega|<=" << max_omega
                  << "; duration_s must be greater than zero." << std::endl;
        return 1;
    }

#ifdef X30_TEST_WITH_ROS1
    // 初始化 HAL 前先记录当前步态和起始里程计，
    // 确保首个速度包发送时反馈已就绪。
    ros::NodeHandle node;
    IntTracker gait_tracker;
    OdomTracker odom_tracker;
    VelocityTracker velocity_tracker;
    const auto gait_sub = node.subscribe<std_msgs::Int32>(
        "/robot_gait_state", 20, &IntTracker::callback, &gait_tracker);
    const auto odom_sub = node.subscribe<nav_msgs::Odometry>(
        "/leg_odom", 50, &OdomTracker::callback, &odom_tracker);
    const auto velocity_sub = node.subscribe<geometry_msgs::Twist>(
        "/robot_velocity", 50, &VelocityTracker::callback, &velocity_tracker);
    (void)gait_sub;
    (void)odom_sub;
    (void)velocity_sub;

    const auto sample_deadline = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(1000);
    IntSample current_gait;
    OdomSample start_odom;
    while (ros::ok() && std::chrono::steady_clock::now() < sample_deadline) {
        ros::spinOnce();
        gait_tracker.latest(current_gait);
        odom_tracker.latest(start_odom);
        if (current_gait.received_at.time_since_epoch().count() != 0 &&
            start_odom.received_at.time_since_epoch().count() != 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 发送任何非零命令前先检查普通楼梯限幅。
    // HAL 会再次限幅，但在此拒绝可直接暴露测试人员的输入错误。
    if (current_gait.value == 6 &&
        (std::abs(vx) > x30_udp_protocol::kGaitStairsMaxVx ||
         std::abs(vy) > x30_udp_protocol::kGaitStairsMaxVy ||
         std::abs(omega) > x30_udp_protocol::kGaitStairsMaxOmega)) {
        std::cerr
            << "[robot_test_x30] Current gait is ordinary stairs. Official "
            << "limits are |vx|<=0.3, |vy|<=0.2, |omega|<=0.8."
            << std::endl;
        return 1;
    }
#endif

    const auto robot = RobotFactory::RobotAllocate(config);
    const int32_t init_result = robot->initRobotHardware();
    if (init_result != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30] Robot init failed, ret="
                  << init_result << "." << std::endl;
        return 1;
    }

    RobotVelocityCommand command{vx, vy, omega};
    const auto period = std::chrono::milliseconds(1000 / kCommandHz);
    const auto started_at = std::chrono::steady_clock::now();
    const auto end_at = started_at + std::chrono::duration<double>(duration_s);
    std::size_t commands_sent = 0;
    bool command_ok = true;

    std::cout << std::fixed << std::setprecision(3)
              << "[robot_test_x30] Moving: vx=" << vx
              << ", vy=" << vy << ", omega=" << omega
              << ", duration_s=" << duration_s << ", rate="
              << kCommandHz << " Hz." << std::endl;

    // steady_clock 可防止系统时间校正缩短或延长实机命令窗口。
    while (std::chrono::steady_clock::now() < end_at) {
        if (robot->writeRobotVelocityCommand(command) != CMD_SUCCESS) {
            command_ok = false;
            break;
        }
        ++commands_sent;
#ifdef X30_TEST_WITH_ROS1
        ros::spinOnce();
#endif
        std::this_thread::sleep_for(period);
    }

    // 先统计命令运行时间，再发送零速度，
    // 使报告时长不包含运动后的主动保持时间。
    const auto stopped_at = std::chrono::steady_clock::now();
    const bool zero_ok = sendZeroFor(robot, kZeroHoldMs);
    const double elapsed_s =
        std::chrono::duration<double>(stopped_at - started_at).count();

    std::cout << "[robot_test_x30] Move finished: command_ok="
              << std::boolalpha << command_ok
              << ", zero_ok=" << zero_ok
              << ", commands_sent=" << commands_sent
              << ", elapsed_s=" << elapsed_s;

#ifdef X30_TEST_WITH_ROS1
    OdomSample final_odom;
    VelocitySample final_velocity;
    if (odom_tracker.latest(final_odom) &&
        start_odom.received_at.time_since_epoch().count() != 0) {
        std::cout << ", odom_distance_m="
                  << planarDistance(start_odom, final_odom);
    }
    if (velocity_tracker.latest(final_velocity)) {
        std::cout << ", feedback=(" << final_velocity.vx << ", "
                  << final_velocity.vy << ", "
                  << final_velocity.omega << ")";
    }
#endif

    std::cout << "." << std::endl;
    return command_ok && zero_ok ? 0 : 1;
}

}  // namespace

int main(int argc, char **argv)
{
    // 仅启用反馈的构建初始化 ROS；纯数据包 Docker 构建
    // 无需依赖 ROS1，仍保持相同 CLI 与 HAL 行为。
#ifdef X30_TEST_WITH_ROS1
    ros::init(argc, argv, "robot_test_x30");
#endif

    CommandLine parsed;
    if (!parseCommandLine(argc, argv, parsed)) {
        printUsage(argv[0]);
        return 1;
    }

    // help 和 status 不需要配置文件或具备 UDP 能力的机器人对象。
    if (parsed.command == "help" || parsed.command == "--help" ||
        parsed.command == "list") {
        printUsage(argv[0]);
        return 0;
    }

    if (parsed.command == "status") {
        return runStatus();
    }

    // 所有改变状态的路径都在 RobotFactory 分配前校验目标型号，
    // 防止 X30 现场工具控制其他机器人。
    YAML::Node config;
    try {
        config = loadX30Config(parsed.config_path);
    } catch (const std::exception &error) {
        std::cerr << "[robot_test_x30] Failed to load config: "
                  << error.what() << "." << std::endl;
        return 1;
    }

    if (parsed.command == "zero" || parsed.command == "stop") {
        const auto robot = RobotFactory::RobotAllocate(config);
        const int32_t init_result = robot->initRobotHardware();
        if (init_result != CMD_SUCCESS) {
            std::cerr << "[robot_test_x30] Robot init failed, ret="
                      << init_result << "." << std::endl;
            return 1;
        }
        const bool ok = sendZeroFor(robot, kZeroHoldMs);
        std::cout << "[robot_test_x30] Zero velocity sent; gait unchanged."
                  << std::endl;
        return ok ? 0 : 1;
    }

    // 步态别名与 "gait <name>" 在此汇合，只发送一个 action。
    std::string gait_name;
    if (parsed.command == "gait") {
        if (argc != parsed.argument_index + 1) {
            printUsage(argv[0]);
            return 1;
        }
        gait_name = argv[parsed.argument_index];
    } else if (parsed.command == "walk" || parsed.command == "l_walk" ||
               parsed.command == "mountain" || parsed.command == "stairs") {
        if (argc != parsed.argument_index) {
            printUsage(argv[0]);
            return 1;
        }
        gait_name = parsed.command;
    }

    if (!gait_name.empty()) {
        GaitCommand gait{};
        if (!lookupGait(gait_name, gait)) {
            std::cerr << "[robot_test_x30] Unknown gait: "
                      << gait_name << "." << std::endl;
            return 1;
        }
        return runGait(config, gait);
    }

    // 可发送非零速度的运动除 runMove() 内有限值解析和配置限幅检查外，
    // 还要求字面确认 token。
    if (parsed.command == "move") {
        if (argc != parsed.argument_index + 5 ||
            std::string(argv[parsed.argument_index + 4]) !=
                kMoveConfirmation) {
            printUsage(argv[0]);
            return 1;
        }

        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        double duration_s = 0.0;
        if (!parseFiniteDouble(argv[parsed.argument_index], vx) ||
            !parseFiniteDouble(argv[parsed.argument_index + 1], vy) ||
            !parseFiniteDouble(argv[parsed.argument_index + 2], omega) ||
            !parseFiniteDouble(
                argv[parsed.argument_index + 3], duration_s)) {
            std::cerr << "[robot_test_x30] move arguments must be finite "
                      << "numbers." << std::endl;
            return 1;
        }
        return runMove(config, vx, vy, omega, duration_s);
    }

    printUsage(argv[0]);
    return 1;
}
