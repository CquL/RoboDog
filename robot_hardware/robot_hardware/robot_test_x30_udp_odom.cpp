// X30 UDP + ROS1 leg_odom closed-loop distance test.
// This is a test tool, not the production navigation path.
// It sends small UDP velocity commands through DeepRoboticsX30 and stops when
// /leg_odom reports that the planar target distance has been reached.

#include "deep_robotics_x30.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <yaml-cpp/yaml.h>

namespace {

// All X30 velocity backends are exercised at the factory 50 Hz cadence.
constexpr int kCommandHz = 50;
constexpr int kStopCount = 20;
constexpr int kMotionStateSettleMs = 1200;
constexpr double kDefaultTargetDistanceM = 0.30;
constexpr double kDefaultCommandSpeedMps = 0.02;
constexpr double kDefaultMaxDurationS = 15.0;

struct OdomSample {
    double x = 0.0;
    double y = 0.0;
    ros::Time stamp;
};

struct VelocitySample {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct BasicStateSample {
    int value = -1;
    uint64_t sequence = 0;
};

class OdomTracker {
public:
    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_.x = msg->pose.pose.position.x;
        latest_.y = msg->pose.pose.position.y;
        latest_.stamp = msg->header.stamp;
        has_latest_ = true;
    }

    bool latest(OdomSample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_latest_) {
            return false;
        }

        sample = latest_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    OdomSample latest_;
    bool has_latest_ = false;
};

class VelocityTracker {
public:
    void callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_.x = msg->linear.x;
        latest_.y = msg->linear.y;
        latest_.yaw = msg->angular.z;
        has_latest_ = true;
    }

    bool latest(VelocitySample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_latest_) {
            return false;
        }

        sample = latest_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    VelocitySample latest_;
    bool has_latest_ = false;
};

class BasicStateTracker {
public:
    void callback(const std_msgs::Int32::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_.value = msg->data;
        ++latest_.sequence;
        has_latest_ = true;
    }

    bool latest(BasicStateSample &sample) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_latest_) {
            return false;
        }

        sample = latest_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    BasicStateSample latest_;
    bool has_latest_ = false;
};

void printUsage(const char *program)
{
    std::cout << "Usage:\n"
              << "  " << program << " [config.yaml] zero\n"
              << "  " << program << " [config.yaml] forward|backward|left|right "
              << "[target_distance_m] [cmd_speed_mps] [max_duration_s]\n\n"
              << "Examples:\n"
              << "  " << program << " ../config.yaml zero\n"
              << "  " << program << " ../config.yaml forward 0.30 0.02 15\n"
              << "  " << program << " ../config.yaml forward 1.00 0.02 30\n\n"
              << "Defaults:\n"
              << "  target_distance_m=" << kDefaultTargetDistanceM << "\n"
              << "  cmd_speed_mps=" << kDefaultCommandSpeedMps << "\n"
              << "  max_duration_s=" << kDefaultMaxDurationS << "\n\n"
              << "Run zero first. Keep the remote controller ready.\n";
}

bool isDirection(const std::string &mode)
{
    return mode == "forward" || mode == "backward" ||
           mode == "left" || mode == "right";
}

bool backendUsesMotionToggle(const YAML::Node &config)
{
    const std::string backend = config["velocity_backend"]
        ? config["velocity_backend"].as<std::string>()
        : "udp_axis";
    return backend == "udp_axis";
}

double parsePositiveDouble(const char *value, double fallback)
{
    if (value == nullptr) {
        return fallback;
    }

    char *end = nullptr;
    double parsed = std::strtod(value, &end);
    if (end == value || parsed <= 0.0) {
        return fallback;
    }

    return parsed;
}

RobotVelocityCommand makeDirectionCommand(const std::string &mode, double speed_mps)
{
    RobotVelocityCommand cmd{0.0, 0.0, 0.0};

    if (mode == "forward") {
        cmd.vx = speed_mps;
    } else if (mode == "backward") {
        cmd.vx = -speed_mps;
    } else if (mode == "left") {
        cmd.vy = speed_mps;
    } else if (mode == "right") {
        cmd.vy = -speed_mps;
    }

    return cmd;
}

void sendStop(const std::shared_ptr<RobotHardwareInterface> &robot)
{
    RobotVelocityCommand stop_cmd{0.0, 0.0, 0.0};
    for (int i = 0; i < kStopCount; ++i) {
        robot->writeRobotVelocityCommand(stop_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool toggleStartStopMotion(const std::shared_ptr<RobotHardwareInterface> &robot,
                           const std::string &phase)
{
    std::cout << "[robot_test_x30_udp_odom] Sending start/stop motion toggle: "
              << phase << std::endl;
    int32_t ret = robot->writeActionCommand(ACTION_START_STOP_MOTION);
    if (ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_udp_odom] start/stop motion toggle failed, ret="
                  << ret << std::endl;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kMotionStateSettleMs));
    return true;
}

bool waitForOdom(OdomTracker &tracker, OdomSample &sample, double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0));
    ros::Rate rate(50);

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        if (tracker.latest(sample)) {
            return true;
        }
        rate.sleep();
    }

    return false;
}

bool waitForBasicState(BasicStateTracker &tracker,
                       uint64_t minimum_sequence,
                       BasicStateSample &sample,
                       double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0));
    ros::Rate rate(50);

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        if (tracker.latest(sample) && sample.sequence > minimum_sequence) {
            return true;
        }
        rate.sleep();
    }

    return false;
}

void printBasicState(const std::string &label, const BasicStateSample &sample)
{
    std::cout << "[robot_test_x30_udp_odom] Basic state " << label
              << ": " << sample.value;
    if (sample.value != 4) {
        std::cout << " (does not match documented stepping state 4)";
    }
    std::cout << std::endl;
}

double planarDistance(const OdomSample &start, const OdomSample &current)
{
    return std::hypot(current.x - start.x, current.y - start.y);
}

double directionalVelocity(const VelocitySample &sample, const std::string &mode)
{
    if (mode == "forward") {
        return sample.x;
    }
    if (mode == "backward") {
        return -sample.x;
    }
    if (mode == "left") {
        return sample.y;
    }
    if (mode == "right") {
        return -sample.y;
    }

    return 0.0;
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_test_x30_udp_odom");

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string config_path = "../config.yaml";
    std::string mode;
    int first_numeric_arg = 0;

    if (argc > 1 && (std::string(argv[1]) == "zero" ||
                     std::string(argv[1]) == "list" ||
                     std::string(argv[1]) == "help" ||
                     std::string(argv[1]) == "--help" ||
                     isDirection(argv[1]))) {
        mode = argv[1];
        first_numeric_arg = 2;
    } else {
        if (argc < 3) {
            printUsage(argv[0]);
            return 1;
        }

        config_path = argv[1];
        mode = argv[2];
        first_numeric_arg = 3;
    }

    if (mode == "list" || mode == "help" || mode == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    if (mode != "zero" && !isDirection(mode)) {
        std::cerr << "[robot_test_x30_udp_odom] Unknown mode: " << mode << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    const double target_distance_m =
        argc > first_numeric_arg
            ? parsePositiveDouble(argv[first_numeric_arg], kDefaultTargetDistanceM)
            : kDefaultTargetDistanceM;
    const double cmd_speed_mps =
        argc > first_numeric_arg + 1
            ? parsePositiveDouble(argv[first_numeric_arg + 1], kDefaultCommandSpeedMps)
            : kDefaultCommandSpeedMps;
    const double max_duration_s =
        argc > first_numeric_arg + 2
            ? parsePositiveDouble(argv[first_numeric_arg + 2], kDefaultMaxDurationS)
            : kDefaultMaxDurationS;

    YAML::Node config = YAML::LoadFile(config_path);
    if (!config["robot_model"] ||
        config["robot_model"].as<std::string>() != "deep_robotics_x30") {
        std::cerr << "[robot_test_x30_udp_odom] config robot_model must be deep_robotics_x30."
                  << std::endl;
        return 1;
    }

    const bool use_motion_toggle = backendUsesMotionToggle(config);

    std::shared_ptr<RobotHardwareInterface> robot =
        std::make_shared<DeepRoboticsX30>(config);

    int32_t init_ret = robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_udp_odom] Robot init failed, ret="
                  << init_ret << std::endl;
        return 1;
    }

    std::cout << "[robot_test_x30_udp_odom] Initialized. Sending zero velocity first..."
              << std::endl;
    sendStop(robot);

    if (mode == "zero") {
        robot->writeActionCommand(ACTION_STOP_MOVE);
        std::cout << "[robot_test_x30_udp_odom] Zero-only mode finished." << std::endl;
        return 0;
    }

    ros::NodeHandle nh;
    OdomTracker tracker;
    VelocityTracker velocity_tracker;
    BasicStateTracker basic_state_tracker;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/leg_odom", 20, &OdomTracker::callback, &tracker);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::Twist>(
        "/robot_velocity", 20, &VelocityTracker::callback, &velocity_tracker);
    ros::Subscriber basic_state_sub = nh.subscribe<std_msgs::Int32>(
        "/robot_basic_state", 20, &BasicStateTracker::callback, &basic_state_tracker);

    OdomSample start;
    if (!waitForOdom(tracker, start, 5.0)) {
        std::cerr << "[robot_test_x30_udp_odom] Timed out waiting for /leg_odom."
                  << std::endl;
        sendStop(robot);
        robot->writeActionCommand(ACTION_STOP_MOVE);
        return 1;
    }

    RobotVelocityCommand command = makeDirectionCommand(mode, cmd_speed_mps);

    std::cout << "[robot_test_x30_udp_odom] Odom distance test: mode=" << mode
              << ", target_distance_m=" << target_distance_m
              << ", cmd_speed_mps=" << cmd_speed_mps
              << ", max_duration_s=" << max_duration_s << std::endl;
    std::cout << "[robot_test_x30_udp_odom] Start odom: x=" << start.x
              << ", y=" << start.y << std::endl;

    BasicStateSample basic_state_before;
    if (waitForBasicState(basic_state_tracker, 0, basic_state_before, 2.0)) {
        printBasicState(
            use_motion_toggle ? "before motion toggle" : "before velocity test",
            basic_state_before);
    } else {
        std::cerr << "[robot_test_x30_udp_odom] No /robot_basic_state sample before test."
                  << std::endl;
    }

    sendStop(robot);
    if (use_motion_toggle) {
        if (!toggleStartStopMotion(robot, "start test")) {
            sendStop(robot);
            robot->writeActionCommand(ACTION_STOP_MOVE);
            return 1;
        }

        BasicStateSample basic_state_after_start;
        if (waitForBasicState(basic_state_tracker, basic_state_before.sequence,
                              basic_state_after_start, 2.0)) {
            printBasicState("after motion toggle", basic_state_after_start);
        } else {
            std::cerr << "[robot_test_x30_udp_odom] No new /robot_basic_state "
                      << "sample after motion toggle." << std::endl;
        }
    } else {
        std::cout << "[robot_test_x30_udp_odom] Motion toggle skipped for factory "
                  << "velocity backend; set robot mode with the APP/remote."
                  << std::endl;
    }

    const auto period = std::chrono::milliseconds(1000 / kCommandHz);
    const auto command_start = std::chrono::steady_clock::now();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(max_duration_s * 1000.0));
    auto next_print = std::chrono::steady_clock::now();
    double distance_m = 0.0;
    double feedback_velocity_mps = 0.0;
    double feedback_velocity_sum = 0.0;
    int feedback_velocity_count = 0;
    bool reached = false;

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        OdomSample current;
        ros::spinOnce();

        VelocitySample velocity;
        if (velocity_tracker.latest(velocity)) {
            feedback_velocity_mps = directionalVelocity(velocity, mode);
            feedback_velocity_sum += feedback_velocity_mps;
            ++feedback_velocity_count;
        }

        if (tracker.latest(current)) {
            distance_m = planarDistance(start, current);
            if (std::chrono::steady_clock::now() >= next_print) {
                std::cout << "[robot_test_x30_udp_odom] distance_m=" << distance_m
                          << ", current_x=" << current.x
                          << ", current_y=" << current.y
                          << ", feedback_velocity_mps=" << feedback_velocity_mps
                          << std::endl;
                next_print = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            }

            if (distance_m >= target_distance_m) {
                reached = true;
                break;
            }
        }

        int32_t ret = robot->writeRobotVelocityCommand(command);
        if (ret != CMD_SUCCESS) {
            std::cerr << "[robot_test_x30_udp_odom] Velocity command failed, ret="
                      << ret << std::endl;
            break;
        }

        std::this_thread::sleep_for(period);
    }

    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - command_start).count();
    const double avg_odom_speed_mps =
        elapsed_s > 0.0 ? distance_m / elapsed_s : 0.0;
    const double speed_scale =
        cmd_speed_mps > 0.0 ? avg_odom_speed_mps / cmd_speed_mps : 0.0;
    const double avg_feedback_velocity_mps =
        feedback_velocity_count > 0
            ? feedback_velocity_sum / static_cast<double>(feedback_velocity_count)
            : 0.0;

    std::cout << "[robot_test_x30_udp_odom] Stopping. reached="
              << (reached ? "true" : "false")
              << ", final_distance_m=" << distance_m
              << ", elapsed_s=" << elapsed_s
              << ", avg_odom_speed_mps=" << avg_odom_speed_mps
              << ", speed_scale=" << speed_scale
              << ", avg_feedback_velocity_mps=" << avg_feedback_velocity_mps
              << ", last_feedback_velocity_mps=" << feedback_velocity_mps
              << std::endl;

    sendStop(robot);
    robot->writeActionCommand(ACTION_STOP_MOVE);
    if (use_motion_toggle) {
        BasicStateSample basic_state_before_stop;
        if (basic_state_tracker.latest(basic_state_before_stop)) {
            printBasicState("before final motion toggle", basic_state_before_stop);
        }
        toggleStartStopMotion(robot, "finish test");
        BasicStateSample basic_state_after_stop;
        if (waitForBasicState(basic_state_tracker, basic_state_before_stop.sequence,
                              basic_state_after_stop, 2.0)) {
            printBasicState("after final motion toggle", basic_state_after_stop);
        }
    }
    sendStop(robot);

    return reached ? 0 : 2;
}
