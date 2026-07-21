// X30 gait verification using UDP commands plus ROS1 state feedback.
// Nonzero velocity is available only in the explicit mountain_move mode.

#include "deep_robotics_x30.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <yaml-cpp/yaml.h>

namespace {

constexpr int kCommandHz = 50;
constexpr int kInitialStateTimeoutMs = 5000;
constexpr int kGaitStateTimeoutMs = 5000;
constexpr int kZeroBeforeActionMs = 600;
constexpr int kMountainZeroHoldMs = 2000;
constexpr int kMountainMoveSettleMs = 1000;
constexpr int kSensorFeedbackFreshnessMs = 500;
constexpr int kMotionStateFreshnessMs = 2500;
constexpr int kControlModeFreshnessMs = 2500;
constexpr double kMountainMoveMaxVxMps = 0.5;
constexpr double kMountainMoveDefaultMaxDurationS = 30.0;
constexpr double kMinimumMotionDistanceM = 0.005;
constexpr double kMinimumFeedbackSpeedMps = 0.01;
constexpr const char *kMountainMoveConfirmation = "CONFIRM_MOUNTAIN_MOVE";
constexpr int kWalkGaitState = 0;
constexpr int kLWalkGaitState = 32;
constexpr int kMountainGaitState = 33;
constexpr int kSteppingBasicState = 4;
constexpr int kRlBasicState = 16;
constexpr int kNonManualControlMode = 1;

struct StateSample {
    int value = -1;
    uint64_t sequence = 0;
    std::chrono::steady_clock::time_point received_at{};
};

struct OdomSample {
    double x = 0.0;
    double y = 0.0;
    std::chrono::steady_clock::time_point received_at{};
};

struct VelocitySample {
    double x = 0.0;
    std::chrono::steady_clock::time_point received_at{};
};

class Int32Tracker {
public:
    void callback(const std_msgs::Int32::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.value = msg->data;
        ++sample_.sequence;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(StateSample &sample) const
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
    StateSample sample_;
    bool has_sample_ = false;
};

class UInt8Tracker {
public:
    void callback(const std_msgs::UInt8::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.value = static_cast<int>(msg->data);
        ++sample_.sequence;
        sample_.received_at = std::chrono::steady_clock::now();
        has_sample_ = true;
    }

    bool latest(StateSample &sample) const
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
    StateSample sample_;
    bool has_sample_ = false;
};

class OdomTracker {
public:
    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.x = msg->pose.pose.position.x;
        sample_.y = msg->pose.pose.position.y;
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

class VelocityTracker {
public:
    void callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.x = msg->linear.x;
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

bool isModeArgument(const std::string &arg)
{
    return arg == "status" || arg == "zero" || arg == "mountain" ||
           arg == "mountain_move" || arg == "walk" || arg == "list" ||
           arg == "help" || arg == "--help";
}

void printUsage(const char *program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [config.yaml] status\n"
        << "  " << program << " [config.yaml] zero\n"
        << "  " << program << " [config.yaml] mountain\n"
        << "  " << program << " [config.yaml] mountain_move "
        << "<forward_mps> <duration_s> " << kMountainMoveConfirmation << "\n"
        << "  " << program << " [config.yaml] walk\n\n"
        << "Modes:\n"
        << "  status    Read ROS1 state only; send no UDP command.\n"
        << "  zero      Initialize X30 and send zero velocity only.\n"
        << "  mountain  Switch walk -> mountain, verify gait=33, then restore walk.\n"
        << "  mountain_move  State-guarded forward motion in mountain gait.\n"
        << "  walk      Recovery command: request walk gait and verify gait=0.\n\n"
        << "Safety:\n"
        << "  Use normal body height, keep the robot stationary, clear the area,\n"
        << "  and keep the remote controller ready. Only mountain_move sends\n"
        << "  nonzero velocity. No mode toggles start/stop motion.\n"
        << "  mountain_move is limited to 0 < vx <= " << kMountainMoveMaxVxMps
        << " m/s; its duration limit comes from mountain_move_max_duration_s "
        << "in config.yaml.\n";
}

const char *basicStateName(int state)
{
    switch (state) {
        case kSteppingBasicState:
            return "stepping";
        case kRlBasicState:
            return "RL";
        default:
            return "other";
    }
}

const char *gaitStateName(int state)
{
    switch (state) {
        case kWalkGaitState:
            return "walk";
        case 1:
            return "obstacle";
        case 2:
            return "slope";
        case 3:
            return "run";
        case 6:
            return "stairs";
        case 7:
            return "stairs_accumulated";
        case 8:
            return "stairs_45";
        case kLWalkGaitState:
            return "L_walk";
        case kMountainGaitState:
            return "mountain";
        case 34:
            return "silent";
        default:
            return "unknown";
    }
}

void printStates(const StateSample &basic, const StateSample &gait)
{
    std::cout << "[robot_test_x30_gait] /robot_basic_state=" << basic.value
              << " (" << basicStateName(basic.value) << "), "
              << "/robot_gait_state=" << gait.value
              << " (" << gaitStateName(gait.value) << ")" << std::endl;
}

void printControlMode(const StateSample &control_mode)
{
    std::cout << "[robot_test_x30_gait] /control_mode="
              << control_mode.value << " ("
              << (control_mode.value == kNonManualControlMode
                      ? "non-manual"
                      : "manual/unknown")
              << ")" << std::endl;
}

bool waitForInitialStates(Int32Tracker &basic_tracker,
                          Int32Tracker &gait_tracker,
                          StateSample &basic,
                          StateSample &gait)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kInitialStateTimeoutMs);
    ros::Rate rate(50);

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        if (basic_tracker.latest(basic) && gait_tracker.latest(gait)) {
            return true;
        }
        rate.sleep();
    }

    return false;
}

bool waitForControlMode(UInt8Tracker &tracker, StateSample &control_mode)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kInitialStateTimeoutMs);
    ros::Rate rate(50);

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        if (tracker.latest(control_mode)) {
            return true;
        }
        rate.sleep();
    }

    return false;
}

bool waitForGaitState(Int32Tracker &tracker,
                      uint64_t minimum_sequence,
                      int expected_state,
                      StateSample &result)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kGaitStateTimeoutMs);
    ros::Rate rate(50);

    while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
        ros::spinOnce();
        StateSample sample;
        if (tracker.latest(sample) && sample.sequence > minimum_sequence) {
            result = sample;
            if (sample.value == expected_state) {
                return true;
            }
        }
        rate.sleep();
    }

    return false;
}

bool sendZeroFor(const std::shared_ptr<RobotHardwareInterface> &robot,
                 int duration_ms)
{
    RobotVelocityCommand zero{0.0, 0.0, 0.0};
    const int iterations = std::max(1, duration_ms * kCommandHz / 1000);
    const auto period = std::chrono::milliseconds(1000 / kCommandHz);

    for (int i = 0; i < iterations; ++i) {
        ros::spinOnce();
        if (robot->writeRobotVelocityCommand(zero) != CMD_SUCCESS) {
            return false;
        }
        std::this_thread::sleep_for(period);
    }

    return true;
}

void bestEffortStop(const std::shared_ptr<RobotHardwareInterface> &robot)
{
    sendZeroFor(robot, kZeroBeforeActionMs);
    robot->writeActionCommand(ACTION_STOP_MOVE);
}

bool requestGait(const std::shared_ptr<RobotHardwareInterface> &robot,
                 Int32Tracker &gait_tracker,
                 const StateSample &before,
                 const std::string &action,
                 int expected_state,
                 StateSample &after)
{
    if (!sendZeroFor(robot, kZeroBeforeActionMs)) {
        std::cerr << "[robot_test_x30_gait] Failed to send zero velocity before gait command."
                  << std::endl;
        return false;
    }

    const int32_t action_ret = robot->writeActionCommand(action);
    if (action_ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_gait] Gait action failed, ret="
                  << action_ret << std::endl;
        return false;
    }

    if (!waitForGaitState(
            gait_tracker, before.sequence, expected_state, after)) {
        std::cerr << "[robot_test_x30_gait] Timed out waiting for "
                  << "/robot_gait_state=" << expected_state << "." << std::endl;
        return false;
    }

    std::cout << "[robot_test_x30_gait] Verified /robot_gait_state="
              << after.value << " (" << gaitStateName(after.value) << ")."
              << std::endl;
    return true;
}

bool isMotionCapableBasicState(int state)
{
    return state == kSteppingBasicState || state == kRlBasicState;
}

bool configChangesRobotMode(const YAML::Node &config)
{
    const bool configure_non_manual = config["configure_non_manual_mode"]
        ? config["configure_non_manual_mode"].as<bool>()
        : true;
    const bool configure_velocity_source =
        config["configure_navigation_velocity_source"]
            ? config["configure_navigation_velocity_source"].as<bool>()
            : true;
    return configure_non_manual || configure_velocity_source;
}

bool parsePositiveDouble(const char *text, double &value)
{
    if (text == nullptr) {
        return false;
    }

    try {
        std::size_t consumed = 0;
        const std::string input(text);
        const double parsed = std::stod(input, &consumed);
        if (consumed != input.size() || !std::isfinite(parsed) || parsed <= 0.0) {
            return false;
        }
        value = parsed;
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

template <typename Sample>
bool sampleIsFresh(const Sample &sample,
                   std::chrono::steady_clock::time_point now,
                   int freshness_ms = kSensorFeedbackFreshnessMs)
{
    if (sample.received_at == std::chrono::steady_clock::time_point{}) {
        return false;
    }
    return now - sample.received_at <=
           std::chrono::milliseconds(freshness_ms);
}

bool stateIsFresh(const StateSample &sample,
                  std::chrono::steady_clock::time_point now)
{
    return sampleIsFresh(sample, now, kMotionStateFreshnessMs);
}

bool runMountainForward(const std::shared_ptr<RobotHardwareInterface> &robot,
                         Int32Tracker &basic_tracker,
                         Int32Tracker &gait_tracker,
                         UInt8Tracker &control_mode_tracker,
                         OdomTracker &odom_tracker,
                        VelocityTracker &velocity_tracker,
                        double forward_mps,
                        double duration_s)
{
    std::cout << "[robot_test_x30_gait] Mountain movement preflight: holding zero for "
              << kMountainMoveSettleMs << " ms." << std::endl;
    if (!sendZeroFor(robot, kMountainMoveSettleMs)) {
        std::cerr << "[robot_test_x30_gait] Failed to hold zero before movement."
                  << std::endl;
        bestEffortStop(robot);
        return false;
    }

    ros::spinOnce();
    StateSample basic;
    StateSample gait;
    StateSample control_mode;
    const auto preflight_now = std::chrono::steady_clock::now();
    const bool has_basic = basic_tracker.latest(basic);
    const bool has_gait = gait_tracker.latest(gait);
    const bool has_control_mode = control_mode_tracker.latest(control_mode);
    const bool basic_fresh = has_basic && stateIsFresh(basic, preflight_now);
    const bool gait_fresh = has_gait && stateIsFresh(gait, preflight_now);
    const bool control_mode_fresh = has_control_mode && sampleIsFresh(
        control_mode, preflight_now, kControlModeFreshnessMs);
    if (has_control_mode) {
        printControlMode(control_mode);
    }
    if (has_basic && has_gait) {
        printStates(basic, gait);
    }
    if (!basic_fresh || !gait_fresh || !control_mode_fresh ||
        !isMotionCapableBasicState(basic.value) ||
        control_mode.value != kNonManualControlMode ||
        gait.value != kMountainGaitState) {
        std::cerr
            << "[robot_test_x30_gait] Movement preflight failed: state missing, stale, "
            << "or no longer control_mode=1, basic=4/16 and gait=33."
            << std::endl;
        bestEffortStop(robot);
        return false;
    }

    OdomSample odom_start;
    if (!odom_tracker.latest(odom_start) ||
        !sampleIsFresh(odom_start, preflight_now)) {
        std::cerr
            << "[robot_test_x30_gait] Movement preflight failed: /leg_odom is "
            << "missing or stale." << std::endl;
        bestEffortStop(robot);
        return false;
    }

    std::cout << "[robot_test_x30_gait] Starting guarded mountain movement: vx="
              << forward_mps << " m/s, duration=" << duration_s
              << " s, command_rate=" << kCommandHz
              << " Hz, odom_start=(" << odom_start.x << ", "
              << odom_start.y << ")." << std::endl;

    RobotVelocityCommand command{forward_mps, 0.0, 0.0};
    const auto period = std::chrono::milliseconds(1000 / kCommandHz);
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(duration_s));
    auto next_tick = start;
    int commands_sent = 0;
    double feedback_velocity_sum = 0.0;
    int feedback_velocity_count = 0;
    bool completed = true;

    while (std::chrono::steady_clock::now() < deadline) {
        if (!ros::ok()) {
            std::cerr << "[robot_test_x30_gait] ROS shutdown requested during movement."
                      << std::endl;
            completed = false;
            break;
        }

        ros::spinOnce();
        const auto now = std::chrono::steady_clock::now();
        if (!basic_tracker.latest(basic) || !gait_tracker.latest(gait) ||
            !control_mode_tracker.latest(control_mode) ||
            !stateIsFresh(basic, now) || !stateIsFresh(gait, now) ||
            !sampleIsFresh(control_mode, now, kControlModeFreshnessMs)) {
            std::cerr << "[robot_test_x30_gait] State feedback became stale; stopping."
                      << std::endl;
            completed = false;
            break;
        }
        if (!isMotionCapableBasicState(basic.value) ||
            control_mode.value != kNonManualControlMode ||
            gait.value != kMountainGaitState) {
            std::cerr << "[robot_test_x30_gait] State interlock opened; stopping."
                      << std::endl;
            printControlMode(control_mode);
            printStates(basic, gait);
            completed = false;
            break;
        }

        VelocitySample velocity;
        if (velocity_tracker.latest(velocity) && sampleIsFresh(velocity, now)) {
            feedback_velocity_sum += velocity.x;
            ++feedback_velocity_count;
        }

        const int32_t ret = robot->writeRobotVelocityCommand(command);
        if (ret != CMD_SUCCESS) {
            std::cerr << "[robot_test_x30_gait] Velocity write failed, ret="
                      << ret << "; stopping." << std::endl;
            completed = false;
            break;
        }
        ++commands_sent;

        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    OdomSample odom_at_command_end;
    const bool has_command_end_odom = odom_tracker.latest(odom_at_command_end);
    const double command_distance_m = has_command_end_odom
        ? std::hypot(
              odom_at_command_end.x - odom_start.x,
              odom_at_command_end.y - odom_start.y)
        : 0.0;
    const double avg_odom_speed_mps =
        elapsed_s > 0.0 ? command_distance_m / elapsed_s : 0.0;
    const double avg_feedback_velocity_mps = feedback_velocity_count > 0
        ? feedback_velocity_sum / static_cast<double>(feedback_velocity_count)
        : 0.0;
    const bool motion_detected =
        command_distance_m >= kMinimumMotionDistanceM ||
        std::abs(avg_feedback_velocity_mps) >= kMinimumFeedbackSpeedMps;
    const bool movement_success = completed && motion_detected;
    if (completed && !motion_detected) {
        std::cerr
            << "[robot_test_x30_gait] No motion detected by /leg_odom or "
            << "/robot_velocity; treating the movement test as failed."
            << std::endl;
    }
    std::cout << "[robot_test_x30_gait] Mountain movement stopping: command_loop_completed="
              << (completed ? "true" : "false")
              << ", motion_detected=" << (motion_detected ? "true" : "false")
              << ", movement_success=" << (movement_success ? "true" : "false")
              << ", commands_sent=" << commands_sent
              << ", elapsed_s=" << elapsed_s
              << ", command_distance_m=" << command_distance_m
              << ", avg_odom_speed_mps=" << avg_odom_speed_mps
              << ", avg_feedback_velocity_mps=" << avg_feedback_velocity_mps
              << "." << std::endl;
    bestEffortStop(robot);

    ros::spinOnce();
    OdomSample odom_after_stop;
    if (odom_tracker.latest(odom_after_stop)) {
        const double final_distance_m = std::hypot(
            odom_after_stop.x - odom_start.x,
            odom_after_stop.y - odom_start.y);
        std::cout << "[robot_test_x30_gait] Distance after zero/stop hold: "
                  << final_distance_m << " m." << std::endl;
    }
    return movement_success;
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_test_x30_gait");

    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string config_path = "../config.yaml";
    std::string mode;
    int first_mode_argument = 0;
    if (isModeArgument(argv[1])) {
        mode = argv[1];
        first_mode_argument = 2;
    } else if (argc >= 3) {
        config_path = argv[1];
        mode = argv[2];
        first_mode_argument = 3;
    } else {
        printUsage(argv[0]);
        return 1;
    }

    if (!isModeArgument(mode)) {
        std::cerr << "[robot_test_x30_gait] Unknown mode: " << mode << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    if (mode == "list" || mode == "help" || mode == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    double mountain_move_vx = 0.0;
    double mountain_move_duration_s = 0.0;
    if (mode == "mountain_move") {
        if (argc != first_mode_argument + 3 ||
            !parsePositiveDouble(argv[first_mode_argument], mountain_move_vx) ||
            !parsePositiveDouble(
                argv[first_mode_argument + 1], mountain_move_duration_s) ||
            std::string(argv[first_mode_argument + 2]) !=
                kMountainMoveConfirmation) {
            std::cerr
                << "[robot_test_x30_gait] mountain_move requires forward speed, "
                << "duration, and the exact confirmation token."
                << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        if (mountain_move_vx > kMountainMoveMaxVxMps) {
            std::cerr
                << "[robot_test_x30_gait] Refusing mountain_move above "
                << kMountainMoveMaxVxMps << " m/s." << std::endl;
            return 1;
        }
    }

    if (mode == "status") {
        ros::NodeHandle nh;
        Int32Tracker basic_tracker;
        Int32Tracker gait_tracker;
        UInt8Tracker control_mode_tracker;
        ros::Subscriber basic_sub = nh.subscribe<std_msgs::Int32>(
            "/robot_basic_state", 20, &Int32Tracker::callback, &basic_tracker);
        ros::Subscriber gait_sub = nh.subscribe<std_msgs::Int32>(
            "/robot_gait_state", 20, &Int32Tracker::callback, &gait_tracker);
        ros::Subscriber control_mode_sub = nh.subscribe<std_msgs::UInt8>(
            "/control_mode", 20, &UInt8Tracker::callback, &control_mode_tracker);
        (void)basic_sub;
        (void)gait_sub;
        (void)control_mode_sub;

        StateSample basic;
        StateSample gait;
        StateSample control_mode;
        if (!waitForInitialStates(basic_tracker, gait_tracker, basic, gait)) {
            std::cerr << "[robot_test_x30_gait] Timed out waiting for ROS1 state topics."
                      << std::endl;
            return 1;
        }
        if (!waitForControlMode(control_mode_tracker, control_mode)) {
            std::cerr << "[robot_test_x30_gait] Timed out waiting for /control_mode."
                      << std::endl;
            return 1;
        }
        printControlMode(control_mode);
        printStates(basic, gait);
        std::cout << "[robot_test_x30_gait] Status mode sent no UDP command."
                  << std::endl;
        return 0;
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const std::exception &error) {
        std::cerr << "[robot_test_x30_gait] Failed to read config: "
                  << error.what() << std::endl;
        return 1;
    }

    if (!config["robot_model"] ||
        config["robot_model"].as<std::string>() != "deep_robotics_x30") {
        std::cerr << "[robot_test_x30_gait] config robot_model must be deep_robotics_x30."
                  << std::endl;
        return 1;
    }

    if (mode == "mountain_move") {
        const std::string backend = config["velocity_backend"]
            ? config["velocity_backend"].as<std::string>()
            : "udp_axis";
        const std::string command_source = config["motion_command_source"]
            ? config["motion_command_source"].as<std::string>()
            : "navigation";
        const double configured_max_vx = config["max_vx"]
            ? config["max_vx"].as<double>()
            : 0.0;
        const double configured_max_duration_s =
            config["mountain_move_max_duration_s"]
                ? config["mountain_move_max_duration_s"].as<double>()
                : kMountainMoveDefaultMaxDurationS;
        if (backend != "udp_navigation" || command_source != "navigation") {
            std::cerr
                << "[robot_test_x30_gait] mountain_move requires "
                << "velocity_backend=udp_navigation and "
                << "motion_command_source=navigation." << std::endl;
            return 1;
        }
        if (configured_max_vx <= 0.0 || mountain_move_vx > configured_max_vx) {
            std::cerr
                << "[robot_test_x30_gait] Requested vx exceeds config max_vx="
                << configured_max_vx << "." << std::endl;
            return 1;
        }
        if (!std::isfinite(configured_max_duration_s) ||
            configured_max_duration_s <= 0.0) {
            std::cerr
                << "[robot_test_x30_gait] mountain_move_max_duration_s must be "
                << "finite and greater than zero." << std::endl;
            return 1;
        }
        if (mountain_move_duration_s > configured_max_duration_s) {
            std::cerr
                << "[robot_test_x30_gait] Refusing duration above config "
                << "mountain_move_max_duration_s="
                << configured_max_duration_s << " s." << std::endl;
            return 1;
        }
    }

    if (mode != "zero" && configChangesRobotMode(config)) {
        std::cerr
            << "[robot_test_x30_gait] Refusing gait test because init would change "
            << "the robot mode or navigation velocity source. Set both "
            << "configure_non_manual_mode and "
            << "configure_navigation_velocity_source to false for this test."
            << std::endl;
        return 1;
    }

    ros::NodeHandle nh;
    Int32Tracker basic_tracker;
    Int32Tracker gait_tracker;
    UInt8Tracker control_mode_tracker;
    OdomTracker odom_tracker;
    VelocityTracker velocity_tracker;
    ros::Subscriber basic_sub;
    ros::Subscriber gait_sub;
    ros::Subscriber control_mode_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber velocity_sub;
    StateSample basic_before;
    StateSample gait_before;
    StateSample control_mode_before;

    if (mode != "zero") {
        basic_sub = nh.subscribe<std_msgs::Int32>(
            "/robot_basic_state", 20, &Int32Tracker::callback, &basic_tracker);
        gait_sub = nh.subscribe<std_msgs::Int32>(
            "/robot_gait_state", 20, &Int32Tracker::callback, &gait_tracker);
        if (mode == "mountain_move") {
            control_mode_sub = nh.subscribe<std_msgs::UInt8>(
                "/control_mode", 20, &UInt8Tracker::callback,
                &control_mode_tracker);
            odom_sub = nh.subscribe<nav_msgs::Odometry>(
                "/leg_odom", 50, &OdomTracker::callback, &odom_tracker);
            velocity_sub = nh.subscribe<geometry_msgs::Twist>(
                "/robot_velocity", 50, &VelocityTracker::callback,
                &velocity_tracker);
        }

        if (!waitForInitialStates(
                basic_tracker, gait_tracker, basic_before, gait_before)) {
            std::cerr << "[robot_test_x30_gait] Timed out waiting for ROS1 state topics."
                      << std::endl;
            return 1;
        }
        printStates(basic_before, gait_before);

        if (mode == "mountain_move") {
            if (!waitForControlMode(control_mode_tracker, control_mode_before)) {
                std::cerr
                    << "[robot_test_x30_gait] Timed out waiting for /control_mode."
                    << std::endl;
                return 1;
            }
            printControlMode(control_mode_before);
            if (control_mode_before.value != kNonManualControlMode) {
                std::cerr
                    << "[robot_test_x30_gait] Refusing mountain movement: "
                    << "APP/remote must remain in non-manual navigation mode."
                    << std::endl;
                return 1;
            }
        }

        if (!isMotionCapableBasicState(basic_before.value)) {
            std::cerr
                << "[robot_test_x30_gait] Refusing gait command: basic state must be "
                << "documented stepping (4) or the observed X30 RL state (16). "
                << "Use the APP/remote to enter the proper stationary motion state."
                << std::endl;
            return 1;
        }

        const bool requests_mountain =
            mode == "mountain" || mode == "mountain_move";
        if (requests_mountain) {
            std::cout
                << "[robot_test_x30_gait] Current gait=" << gait_before.value
                << " (" << gaitStateName(gait_before.value) << "). "
                << "Requesting mountain directly; no intermediate walk gait is required."
                << std::endl;
        }
    }

    std::shared_ptr<RobotHardwareInterface> robot =
        std::make_shared<DeepRoboticsX30>(config);
    const int32_t init_ret = robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_gait] Robot init failed, ret="
                  << init_ret << std::endl;
        return 1;
    }

    std::cout << "[robot_test_x30_gait] Initialized. Sending zero velocity first."
              << std::endl;
    bestEffortStop(robot);

    if (mode == "zero") {
        std::cout << "[robot_test_x30_gait] Zero-only mode finished."
                  << std::endl;
        return 0;
    }

    if (mode == "walk" && gait_before.value == kWalkGaitState) {
        std::cout << "[robot_test_x30_gait] Robot is already in walk gait."
                  << std::endl;
        bestEffortStop(robot);
        return 0;
    }

    const bool requests_mountain =
        mode == "mountain" || mode == "mountain_move";
    const std::string requested_action =
        requests_mountain ? ACTION_GAIT_MOUNTAIN : ACTION_GAIT_WALK;
    const int requested_state =
        requests_mountain ? kMountainGaitState : kWalkGaitState;
    StateSample gait_after_request;
    const bool request_ok = requestGait(
        robot,
        gait_tracker,
        gait_before,
        requested_action,
        requested_state,
        gait_after_request);

    if (mode == "walk") {
        bestEffortStop(robot);
        return request_ok ? 0 : 1;
    }

    bool operation_ok = request_ok;
    if (request_ok && mode == "mountain_move") {
        operation_ok = runMountainForward(
            robot,
            basic_tracker,
            gait_tracker,
            control_mode_tracker,
            odom_tracker,
            velocity_tracker,
            mountain_move_vx,
            mountain_move_duration_s);
    } else if (request_ok) {
        std::cout
            << "[robot_test_x30_gait] Mountain gait verified while stationary. "
            << "Holding zero velocity before automatic L-walk recovery."
            << std::endl;
        operation_ok = sendZeroFor(robot, kMountainZeroHoldMs);
    } else {
        std::cerr
            << "[robot_test_x30_gait] Mountain verification failed. Attempting "
            << "L-walk recovery before exit." << std::endl;
    }

    StateSample gait_before_recovery;
    if (!gait_tracker.latest(gait_before_recovery)) {
        gait_before_recovery = gait_before;
    }
    StateSample gait_after_recovery;
    const bool recovery_ok = requestGait(
        robot,
        gait_tracker,
        gait_before_recovery,
        ACTION_GAIT_L_WALK,
        kLWalkGaitState,
        gait_after_recovery);
    bestEffortStop(robot);

    if (!recovery_ok) {
        std::cerr
            << "[robot_test_x30_gait] Automatic L-walk recovery was not confirmed. "
            << "Keep the robot stopped and use the APP/remote."
            << std::endl;
        return 2;
    }

    std::cout << "[robot_test_x30_gait] Mountain test finished; L-walk gait restored."
              << std::endl;
    return operation_ok ? 0 : 1;
}
