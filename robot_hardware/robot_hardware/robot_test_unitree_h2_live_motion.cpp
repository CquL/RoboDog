#include "robot_factory.h"
#include "unitree/unitree_h2.h"
#include "unitree/unitree_h2_live_motion_plan.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace {

using unitree_h2_live_motion::AxisPlan;

constexpr const char *kAcknowledgement =
    "H2_FALL_ARREST_STAND_FOUR_CASTERS_LOCKED_CLEAR_AREA_SECOND_OPERATOR_REMOTE_HIGH_LEVEL_STOP_CONFIRMED";
constexpr const char *kAuthorizationPrefix =
    "/home/unitree/p2_unitreeH2/build/h2_control_gate_state/"
    ".stage06e_authorization_";

volatile std::sig_atomic_t g_stop_requested = 0;

void onSignal(int)
{
    g_stop_requested = 1;
}

struct Options {
    std::string config_path;
    std::string axis;
    std::string acknowledgement;
    std::string authorization_file;
    int expected_fsm = -1;
    double linear_speed = unitree_h2_live_motion::kDefaultLinearSpeed;
    double yaw_speed = unitree_h2_live_motion::kDefaultYawSpeed;
    int stream_ms = unitree_h2_live_motion::kDefaultStreamMilliseconds;
    bool linear_speed_set = false;
    bool yaw_speed_set = false;
    bool stream_ms_set = false;
    bool live_motion = false;
    bool print_plan = false;
    bool help = false;
};

void printUsage(const char *program)
{
    std::cerr
        << "Usage:\n"
        << "  " << program << " --print-plan --axis <axis>\n"
        << "  " << program
        << " --config <yaml> --axis <axis> --expected-fsm 601"
           " --live-motion --acknowledge "
        << kAcknowledgement
        << " --authorization-file <one-time-stage06e-file>\n\n"
        << "Allowed uncalibrated axes: x-positive, x-negative, y-positive, "
           "y-negative, yaw-positive, yaw-negative\n"
        << "Optional bounded stream profile: --linear-speed <0.01..0.10> "
           "--yaw-speed <0.01..0.15> --stream-ms <250..1000>\n"
        << "The stream duration must use 50 ms increments. Commands are sent "
           "at a fixed 20 Hz through writeRobotVelocityCommand().\n"
        << "Speed values use 0.001 increments.\n"
        << "Only one axis is sent per invocation. Human direction names such "
           "as forward/left are intentionally rejected until sign calibration."
        << std::endl;
}

bool parseInteger(const std::string &text, int &value)
{
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseDouble(const std::string &text, double &value)
{
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool requireValue(int argc, char *argv[], int &index, std::string &value)
{
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return true;
}

bool parseOptions(int argc, char *argv[], Options &options)
{
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            options.help = true;
        } else if (argument == "--live-motion") {
            options.live_motion = true;
        } else if (argument == "--print-plan") {
            options.print_plan = true;
        } else if (argument == "--config") {
            if (!requireValue(argc, argv, index, options.config_path)) {
                return false;
            }
        } else if (argument == "--axis") {
            if (!requireValue(argc, argv, index, options.axis)) {
                return false;
            }
        } else if (argument == "--acknowledge") {
            if (!requireValue(argc, argv, index, options.acknowledgement)) {
                return false;
            }
        } else if (argument == "--authorization-file") {
            if (!requireValue(argc, argv, index,
                              options.authorization_file)) {
                return false;
            }
        } else if (argument == "--expected-fsm") {
            std::string text;
            if (!requireValue(argc, argv, index, text) ||
                !parseInteger(text, options.expected_fsm)) {
                return false;
            }
        } else if (argument == "--linear-speed") {
            std::string text;
            if (options.linear_speed_set ||
                !requireValue(argc, argv, index, text) ||
                !parseDouble(text, options.linear_speed)) {
                return false;
            }
            options.linear_speed_set = true;
        } else if (argument == "--yaw-speed") {
            std::string text;
            if (options.yaw_speed_set ||
                !requireValue(argc, argv, index, text) ||
                !parseDouble(text, options.yaw_speed)) {
                return false;
            }
            options.yaw_speed_set = true;
        } else if (argument == "--stream-ms") {
            std::string text;
            if (options.stream_ms_set ||
                !requireValue(argc, argv, index, text) ||
                !parseInteger(text, options.stream_ms)) {
                return false;
            }
            options.stream_ms_set = true;
        } else {
            return false;
        }
    }
    return true;
}

bool consumeOneTimeAuthorization(const std::string &path)
{
    const char *environment_token = std::getenv("H2_LIVE_GATE_TOKEN");
    if (environment_token == nullptr) {
        std::cerr << "Missing one-time Stage 06E authorization token."
                  << std::endl;
        return false;
    }
    const std::string token(environment_token);
    const bool token_is_lower_hex =
        token.size() == 64 &&
        std::all_of(token.begin(), token.end(), [](unsigned char value) {
            return (value >= '0' && value <= '9') ||
                   (value >= 'a' && value <= 'f');
        });
    if (!token_is_lower_hex || path != std::string(kAuthorizationPrefix) + token) {
        std::cerr << "Invalid Stage 06E authorization path or token format."
                  << std::endl;
        return false;
    }

    struct stat status {};
    if (lstat(path.c_str(), &status) != 0 || !S_ISREG(status.st_mode) ||
        status.st_uid != geteuid() || status.st_nlink != 1 ||
        (status.st_mode & 0777) != 0600) {
        std::cerr << "Stage 06E authorization file ownership/type/mode rejected."
                  << std::endl;
        return false;
    }

    std::ifstream input(path);
    std::string file_token;
    std::string unexpected_second_line;
    if (!input || !std::getline(input, file_token) ||
        std::getline(input, unexpected_second_line) || file_token != token) {
        std::cerr << "Stage 06E authorization file content rejected."
                  << std::endl;
        return false;
    }
    input.close();

    if (std::remove(path.c_str()) != 0) {
        std::cerr << "Failed to consume the one-time Stage 06E authorization."
                  << std::endl;
        return false;
    }
    unsetenv("H2_LIVE_GATE_TOKEN");
    std::cout << "H2_LIVE_ONE_TIME_AUTHORIZATION_CONSUMED" << std::endl;
    return true;
}

void printPlan(const AxisPlan &plan)
{
    using namespace unitree_h2_live_motion;
    std::cout << std::fixed << std::setprecision(3)
              << "H2_LIVE_PLAN axis=" << plan.axis
              << " vx=" << plan.command.vx
              << " vy=" << plan.command.vy
              << " omega=" << plan.command.omega
              << " linear_speed=" << plan.linear_speed
              << " yaw_speed=" << plan.yaw_speed
              << " vendor_duration_s=" << plan.vendor_duration_s
              << " stream_ms=" << plan.stream_ms
              << " command_hz=" << plan.command_hz
              << " command_period_ms=" << plan.command_period_ms
              << " max_send_gap_ms=" << plan.max_send_gap_ms
              << " expected_rpc_count=" << plan.expected_rpc_count
              << " watchdog_ms=" << plan.watchdog_ms
              << " sdk_timeout_s=" << kSdkTimeoutS
              << " expected_fsm=" << kInitialMotionFsmId << std::endl;
}

bool interruptibleSleep(std::chrono::milliseconds duration)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (!g_stop_requested && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return g_stop_requested == 0;
}

class StopGuard {
public:
    explicit StopGuard(std::shared_ptr<RobotHardwareInterface> robot)
        : robot_(std::move(robot))
    {
    }

    ~StopGuard()
    {
        if (robot_) {
            const int32_t result = stopNow("scope-exit");
            if (result != CMD_SUCCESS) {
                std::cerr << "H2_LIVE_STOP_GUARD_FAILED code=" << result
                          << std::endl;
            }
        }
    }

    int32_t stopNow(const char *reason, int max_attempts = 1)
    {
        int32_t last_result = ERROR_ROBOT_HARDWARE_MOVE;
        for (int attempt = 1; attempt <= std::max(1, max_attempts); ++attempt) {
            const int32_t stop_result =
                robot_->writeActionCommand(ACTION_STOP_MOVE);
            RobotVelocityCommand zero{0.0, 0.0, 0.0};
            const int32_t zero_result = robot_->writeRobotVelocityCommand(zero);
            std::cout << "H2_LIVE_STOP reason=" << reason
                      << " attempt=" << attempt
                      << " stop_move_ret=" << stop_result
                      << " zero_velocity_ret=" << zero_result << std::endl;
            if (stop_result == CMD_SUCCESS && zero_result == CMD_SUCCESS) {
                return CMD_SUCCESS;
            }
            last_result = stop_result != CMD_SUCCESS ? stop_result : zero_result;
            if (attempt < max_attempts) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        return last_result;
    }

private:
    std::shared_ptr<RobotHardwareInterface> robot_;
};

bool containsFsm(const std::vector<int> &ids, int expected)
{
    return std::find(ids.begin(), ids.end(), expected) != ids.end();
}

void printAvailableFsms(const std::vector<int> &ids,
                        const std::vector<std::string> &names)
{
    std::cout << "H2_LIVE_AVAILABLE_FSMS count=" << ids.size();
    for (std::size_t index = 0; index < ids.size(); ++index) {
        std::cout << " [" << ids[index];
        if (index < names.size()) {
            std::cout << ":" << names[index];
        }
        std::cout << "]";
    }
    std::cout << std::endl;
}

int failWithStop(StopGuard &stop_guard, int exit_code, const char *reason)
{
    stop_guard.stopNow(reason);
    return exit_code;
}

} // namespace

int main(int argc, char *argv[])
{
    using namespace unitree_h2_live_motion;

    Options options;
    if (!parseOptions(argc, argv, options)) {
        printUsage(argv[0]);
        return 64;
    }
    if (options.help) {
        printUsage(argv[0]);
        return 0;
    }

    const auto plan = planForAxis(options.axis, options.linear_speed,
                                  options.yaw_speed, options.stream_ms);
    if (!plan.has_value() || nonZeroAxisCount(plan->command) != 1) {
        std::cerr << "Invalid or non-single-axis motion plan." << std::endl;
        printUsage(argv[0]);
        return 64;
    }
    printPlan(*plan);
    if (options.print_plan) {
        std::cout << "H2_LIVE_PRINT_PLAN_ONLY_NO_DDS" << std::endl;
        return 0;
    }

    if (!options.live_motion || options.config_path.empty() ||
        options.acknowledgement != kAcknowledgement ||
        options.authorization_file.empty() ||
        options.expected_fsm != kInitialMotionFsmId) {
        std::cerr << "Live-motion interlock rejected the invocation." << std::endl;
        printUsage(argv[0]);
        return 65;
    }

    if (!consumeOneTimeAuthorization(options.authorization_file)) {
        std::cerr << "One-time Stage 06E authorization rejected before DDS "
                     "initialization."
                  << std::endl;
        return 65;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);
    std::signal(SIGQUIT, onSignal);

    YAML::Node config;
    try {
        config = YAML::LoadFile(options.config_path);
    } catch (const std::exception &error) {
        std::cerr << "Failed to load H2 config: " << error.what() << std::endl;
        return 66;
    }
    if (!config["robot_model"] ||
        config["robot_model"].as<std::string>() != "unitree_h2") {
        std::cerr << "Config is not for robot_model=unitree_h2." << std::endl;
        return 66;
    }

    // Never persist an unlocked YAML. This process-local override is reachable
    // only after all command-line interlocks above pass.
    config["allow_motion_commands"] = true;
    config["allow_state_changing_actions"] = false;
    config["verify_fsm_on_init"] = true;
    config["sdk_timeout_s"] = kSdkTimeoutS;
    config["max_vx"] = plan->linear_speed;
    config["max_vy"] = plan->linear_speed;
    config["max_omega"] = plan->yaw_speed;
    config["velocity_command_duration_s"] = plan->vendor_duration_s;
    config["velocity_watchdog_hz"] = 100;
    config["velocity_command_timeout_ms"] = plan->watchdog_ms;
    config["velocity_zero_hold_ms"] = kZeroHoldMilliseconds;
    config["required_motion_fsm_id"] = kInitialMotionFsmId;

    std::shared_ptr<RobotHardwareInterface> robot;
    try {
        robot = RobotFactory::RobotAllocate(config);
    } catch (const std::exception &error) {
        std::cerr << "RobotFactory failed: " << error.what() << std::endl;
        return 67;
    }
    const std::shared_ptr<UnitreeH2> h2 =
        std::dynamic_pointer_cast<UnitreeH2>(robot);
    if (!h2) {
        std::cerr << "RobotFactory did not return UnitreeH2." << std::endl;
        return 67;
    }

    const int32_t init_result = robot->initRobotHardware();
    if (init_result != CMD_SUCCESS) {
        std::cerr << "H2_LIVE_INIT_FAILED code=" << init_result << std::endl;
        return 68;
    }
    StopGuard stop_guard(robot);

    std::vector<int> available_ids;
    std::vector<std::string> available_names;
    const int32_t available_result =
        h2->readAvailableFsmIds(available_ids, available_names);
    if (available_result != CMD_SUCCESS) {
        std::cerr << "H2_LIVE_AVAILABLE_FSM_FAILED code=" << available_result
                  << std::endl;
        return failWithStop(stop_guard, 69, "available-fsm-failed");
    }
    printAvailableFsms(available_ids, available_names);

    int current_fsm = -1;
    int current_mode = -1;
    const int32_t fsm_result = h2->readFsmId(current_fsm);
    const int32_t mode_result = h2->readFsmMode(current_mode);
    std::cout << "H2_LIVE_STATE fsm_id=" << current_fsm
              << " fsm_ret=" << fsm_result
              << " fsm_mode=" << current_mode
              << " mode_ret=" << mode_result
              << " mode_value_observation_only=1" << std::endl;
    if (fsm_result != CMD_SUCCESS || mode_result != CMD_SUCCESS ||
        current_fsm != options.expected_fsm ||
        !containsFsm(available_ids, options.expected_fsm)) {
        std::cerr << "FSM/getter readiness gate rejected motion. The mode RPC "
                     "must succeed, but its undocumented numeric value is not "
                     "used as a motion-state assertion."
                  << std::endl;
        return failWithStop(stop_guard, 70, "fsm-gate-rejected");
    }

    if (stop_guard.stopNow("baseline") != CMD_SUCCESS) {
        std::cerr << "Baseline zero/StopMove failed." << std::endl;
        return 71;
    }
    if (!interruptibleSleep(std::chrono::milliseconds(500))) {
        return failWithStop(stop_guard, 130, "signal-before-countdown");
    }
    int fsm_after_baseline = -1;
    if (h2->readFsmId(fsm_after_baseline) != CMD_SUCCESS ||
        fsm_after_baseline != options.expected_fsm) {
        std::cerr << "FSM changed during baseline stop." << std::endl;
        return failWithStop(stop_guard, 72, "fsm-changed-after-baseline");
    }

    std::cout
        << "H2_LIVE_ARMED signs_unverified=1 no_state_change_actions=1"
        << std::endl;
    for (int remaining = kCountdownSeconds; remaining > 0; --remaining) {
        std::cout << "H2_LIVE_COUNTDOWN seconds=" << remaining << std::endl;
        if (!interruptibleSleep(std::chrono::seconds(1))) {
            return failWithStop(stop_guard, 130, "signal-during-countdown");
        }
    }

    if (g_stop_requested) {
        return failWithStop(stop_guard, 130, "signal-before-pre-stream-zero");
    }
    if (stop_guard.stopNow("pre-stream-baseline") != CMD_SUCCESS) {
        std::cerr << "Last-moment zero/StopMove failed." << std::endl;
        return 72;
    }
    if (!interruptibleSleep(std::chrono::milliseconds(100))) {
        return failWithStop(stop_guard, 130, "signal-after-pre-stream-zero");
    }

    // Recheck immediately after the last-moment zero. The HAL performs a
    // second GetFsmId under its command mutex immediately before SetVelocity,
    // closing the remaining process-local time-of-check/time-of-use gap.
    int pre_stream_fsm = -1;
    int pre_stream_mode = -1;
    const int32_t pre_stream_fsm_result = h2->readFsmId(pre_stream_fsm);
    const int32_t pre_stream_mode_result = h2->readFsmMode(pre_stream_mode);
    std::cout << "H2_LIVE_PRE_STREAM_STATE fsm_id=" << pre_stream_fsm
              << " fsm_ret=" << pre_stream_fsm_result
              << " fsm_mode=" << pre_stream_mode
              << " mode_ret=" << pre_stream_mode_result
              << " mode_value_observation_only=1" << std::endl;
    if (pre_stream_fsm_result != CMD_SUCCESS ||
        pre_stream_mode_result != CMD_SUCCESS ||
        pre_stream_fsm != options.expected_fsm) {
        return failWithStop(stop_guard, 72, "pre-stream-fsm-rejected");
    }
    if (g_stop_requested) {
        return failWithStop(stop_guard, 130, "signal-before-motion-rpc");
    }

    RobotVelocityCommand command = plan->command;
    using Clock = std::chrono::steady_clock;
    using Milliseconds = std::chrono::milliseconds;
    const auto stream_start = Clock::now();
    const auto stream_deadline = stream_start + Milliseconds(plan->stream_ms);
    auto previous_send_start = stream_start;
    long long max_send_gap_ms = 0;
    long long max_rpc_latency_ms = 0;
    int rpc_count = 0;
    std::cout << "H2_LIVE_STREAM_BEGIN axis=" << plan->axis
              << " stream_ms=" << plan->stream_ms
              << " command_hz=" << plan->command_hz
              << " expected_rpc_count=" << plan->expected_rpc_count
              << std::endl;

    for (int sequence = 1; sequence <= plan->expected_rpc_count; ++sequence) {
        const auto scheduled = stream_start +
            Milliseconds((sequence - 1) * plan->command_period_ms);
        const auto before_wait = Clock::now();
        if (before_wait < scheduled &&
            !interruptibleSleep(std::chrono::duration_cast<Milliseconds>(
                scheduled - before_wait))) {
            stop_guard.stopNow("signal-during-stream-wait", 3);
            return 130;
        }
        if (g_stop_requested) {
            stop_guard.stopNow("signal-before-stream-rpc", 3);
            return 130;
        }

        const auto rpc_start = Clock::now();
        if (rpc_start >= stream_deadline) {
            std::cerr << "H2_LIVE_STREAM_DEADLINE_REJECT seq=" << sequence
                      << " rpc_count=" << rpc_count << std::endl;
            stop_guard.stopNow("stream-deadline-overrun", 3);
            return 75;
        }
        const long long send_gap_ms = sequence == 1
            ? 0
            : std::chrono::duration_cast<Milliseconds>(
                  rpc_start - previous_send_start).count();
        max_send_gap_ms = std::max(max_send_gap_ms, send_gap_ms);
        if (sequence > 1 && send_gap_ms > plan->max_send_gap_ms) {
            std::cerr << "H2_LIVE_STREAM_GAP_REJECT seq=" << sequence
                      << " gap_ms=" << send_gap_ms
                      << " limit_ms=" << plan->max_send_gap_ms
                      << " rpc_count=" << rpc_count << std::endl;
            stop_guard.stopNow("stream-send-gap-overrun", 3);
            return 75;
        }

        const int32_t motion_result =
            robot->writeRobotVelocityCommand(command);
        const auto rpc_end = Clock::now();
        const long long rpc_latency_ms =
            std::chrono::duration_cast<Milliseconds>(rpc_end - rpc_start)
                .count();
        max_rpc_latency_ms = std::max(max_rpc_latency_ms, rpc_latency_ms);
        std::cout << "H2_LIVE_STREAM_RPC seq=" << sequence
                  << " ret=" << motion_result
                  << " latency_ms=" << rpc_latency_ms
                  << " gap_ms=" << send_gap_ms << std::endl;
        if (motion_result != CMD_SUCCESS) {
            stop_guard.stopNow("stream-rpc-failed", 3);
            return 73;
        }
        ++rpc_count;
        previous_send_start = rpc_start;
    }

    const auto after_last_rpc = Clock::now();
    bool stream_completed = true;
    if (after_last_rpc < stream_deadline) {
        stream_completed = interruptibleSleep(
            std::chrono::duration_cast<Milliseconds>(
                stream_deadline - after_last_rpc));
    }
    const auto stop_requested_at = Clock::now();
    const long long stream_elapsed_ms =
        std::chrono::duration_cast<Milliseconds>(
            stop_requested_at - stream_start).count();
    const int32_t final_stop_result = stop_guard.stopNow(
        stream_completed ? "stream-complete" : "signal-during-stream", 3);
    const bool zero_hold_completed = interruptibleSleep(
        std::chrono::milliseconds(kZeroHoldMilliseconds));
    if (!zero_hold_completed) {
        stop_guard.stopNow("signal-during-zero-hold");
        return 130;
    }

    int final_fsm = -1;
    const int32_t final_fsm_result = h2->readFsmId(final_fsm);
    std::cout << "H2_LIVE_FINAL_STATE fsm_id=" << final_fsm
              << " fsm_ret=" << final_fsm_result
              << " stop_ret=" << final_stop_result
              << " rpc_count=" << rpc_count
              << " max_observed_send_gap_ms=" << max_send_gap_ms
              << " max_rpc_latency_ms=" << max_rpc_latency_ms
              << " stream_elapsed_ms=" << stream_elapsed_ms << std::endl;
    if (!stream_completed) {
        return 130;
    }
    if (rpc_count != plan->expected_rpc_count ||
        max_send_gap_ms > plan->max_send_gap_ms ||
        final_stop_result != CMD_SUCCESS ||
        final_fsm_result != CMD_SUCCESS ||
        final_fsm != options.expected_fsm) {
        return 74;
    }

    std::cout << "H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK axis=" << plan->axis
              << " rpc_count=" << rpc_count
              << " expected_rpc_count=" << plan->expected_rpc_count
              << " max_observed_send_gap_ms=" << max_send_gap_ms
              << " max_rpc_latency_ms=" << max_rpc_latency_ms
              << " stream_elapsed_ms=" << stream_elapsed_ms
              << " observed_direction_must_be_recorded=1" << std::endl;
    return 0;
}
