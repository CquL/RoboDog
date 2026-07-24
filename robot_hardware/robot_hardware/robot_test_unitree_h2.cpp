#include "robot_factory.h"
#include "unitree/unitree_h2.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kCommandPeriodMs = 50;
constexpr int kMinStreamMs = 100;
constexpr int kMaxStreamMs = 3000;

std::atomic<bool> stop_requested{false};

enum class Mode {
    None,
    ReadOnly,
    GetterAudit,
    ZeroStop,
    Velocity,
    Action,
};

struct Options {
    std::string config_path = "../config/unitree_h2.yaml";
    bool config_path_set = false;
    Mode mode = Mode::None;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    int duration_ms = 0;
    std::string action;
    bool execute = false;
};

void onSignal(int)
{
    stop_requested = true;
}

void printUsage(const char *program)
{
    std::cerr
        << "Usage:\n"
        << "  " << program << " --config <yaml> --read-only\n"
        << "  " << program << " --config <yaml> --getter-audit\n"
        << "  " << program << " --config <yaml> --zero-stop --execute\n"
        << "  " << program
        << " --config <motion.yaml> --velocity --vx <m/s> --vy <m/s>"
           " --omega <rad/s> --duration-ms <100..3000> --execute\n"
        << "  " << program
        << " --config <yaml> --action <name> --execute\n";
}

bool selectMode(Options &options, Mode mode)
{
    if (options.mode != Mode::None) return false;
    options.mode = mode;
    return true;
}

bool parseDouble(const char *text, double &value)
{
    try {
        std::size_t used = 0;
        value = std::stod(text, &used);
        return text[used] == '\0' && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

bool parseInt(const char *text, int &value)
{
    try {
        std::size_t used = 0;
        value = std::stoi(text, &used);
        return text[used] == '\0';
    } catch (...) {
        return false;
    }
}

bool parseOptions(int argc, char *argv[], Options &options)
{
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--read-only") {
            if (!selectMode(options, Mode::ReadOnly)) return false;
        } else if (argument == "--getter-audit") {
            if (!selectMode(options, Mode::GetterAudit)) return false;
        } else if (argument == "--zero-stop") {
            if (!selectMode(options, Mode::ZeroStop)) return false;
        } else if (argument == "--velocity") {
            if (!selectMode(options, Mode::Velocity)) return false;
        } else if (argument == "--execute") {
            options.execute = true;
        } else if (argument == "--config") {
            if (++index >= argc || options.config_path_set) return false;
            options.config_path = argv[index];
            options.config_path_set = true;
        } else if (argument == "--vx") {
            if (++index >= argc || !parseDouble(argv[index], options.vx)) {
                return false;
            }
        } else if (argument == "--vy") {
            if (++index >= argc || !parseDouble(argv[index], options.vy)) {
                return false;
            }
        } else if (argument == "--omega") {
            if (++index >= argc || !parseDouble(argv[index], options.omega)) {
                return false;
            }
        } else if (argument == "--duration-ms") {
            if (++index >= argc || !parseInt(argv[index], options.duration_ms)) {
                return false;
            }
        } else if (argument == "--action") {
            if (++index >= argc || !selectMode(options, Mode::Action)) {
                return false;
            }
            options.action = argv[index];
        } else if (!argument.empty() && argument.front() != '-' &&
                   !options.config_path_set) {
            // Retain the former positional config syntax for old read-only
            // scripts while all new commands use --config explicitly.
            options.config_path = argument;
            options.config_path_set = true;
        } else {
            return false;
        }
    }
    return true;
}

bool validOptions(const Options &options)
{
    if (options.mode == Mode::None || options.config_path.empty()) return false;
    if (options.mode == Mode::ReadOnly || options.mode == Mode::GetterAudit) {
        return !options.execute;
    }
    if (!options.execute) return false;
    if (options.mode == Mode::ZeroStop) return true;
    if (options.mode == Mode::Action) return !options.action.empty();
    return options.duration_ms >= kMinStreamMs &&
           options.duration_ms <= kMaxStreamMs &&
           (options.vx != 0.0 || options.vy != 0.0 || options.omega != 0.0);
}

template <typename T>
T readConfig(const YAML::Node &config, const std::string &key,
             const T &default_value)
{
    return config[key] ? config[key].as<T>() : default_value;
}

bool validateVelocityConfig(const YAML::Node &config, const Options &options)
{
    const bool allow_motion =
        readConfig<bool>(config, "allow_motion_commands", false);
    const bool verify_fsm =
        readConfig<bool>(config, "verify_fsm_on_init", true);
    const double max_vx = readConfig<double>(config, "max_vx", 0.20);
    const double max_vy = readConfig<double>(config, "max_vy", 0.10);
    const double max_omega = readConfig<double>(config, "max_omega", 0.30);

    if (!allow_motion || !verify_fsm) {
        std::cerr << "H2_TEST_CONFIG_REJECTED allow_motion_commands="
                  << allow_motion << " verify_fsm_on_init=" << verify_fsm
                  << std::endl;
        return false;
    }
    if (std::abs(options.vx) > max_vx || std::abs(options.vy) > max_vy ||
        std::abs(options.omega) > max_omega) {
        std::cerr << "H2_TEST_CONFIG_WOULD_CLAMP requested=" << options.vx
                  << ',' << options.vy << ',' << options.omega
                  << " configured_max=" << max_vx << ',' << max_vy << ','
                  << max_omega << std::endl;
        return false;
    }
    return true;
}

int32_t stopRobot(const std::shared_ptr<RobotHardwareInterface> &robot)
{
    const int32_t action_ret = robot->writeActionCommand(ACTION_STOP_MOVE);
    RobotVelocityCommand zero{0.0, 0.0, 0.0};
    const int32_t zero_ret = robot->writeRobotVelocityCommand(zero);
    std::cout << "H2_HAL_STOP action_ret=" << action_ret
              << " zero_ret=" << zero_ret << std::endl;
    return action_ret != CMD_SUCCESS ? action_ret : zero_ret;
}

int runGetterAudit(const std::shared_ptr<UnitreeH2> &h2)
{
    int fsm_id = -1;
    int fsm_mode = -1;
    std::vector<int> ids;
    std::vector<std::string> names;
    const int32_t fsm_ret = h2->readFsmId(fsm_id);
    const int32_t mode_ret = h2->readFsmMode(fsm_mode);
    const int32_t available_ret = h2->readAvailableFsmIds(ids, names);
    std::cout << "H2_GETTER_AUDIT fsm_id=" << fsm_id
              << " fsm_ret=" << fsm_ret << " fsm_mode=" << fsm_mode
              << " mode_ret=" << mode_ret
              << " mode_value_observation_only=1"
              << " available_ret=" << available_ret
              << " available_count=" << ids.size();
    for (std::size_t index = 0; index < ids.size(); ++index) {
        std::cout << " [" << ids[index];
        if (index < names.size()) std::cout << ':' << names[index];
        std::cout << ']';
    }
    std::cout << std::endl;
    if (fsm_ret != CMD_SUCCESS || mode_ret != CMD_SUCCESS ||
        available_ret != CMD_SUCCESS) {
        return 2;
    }
    std::cout << "H2_GETTER_ONLY_RPC_OK" << std::endl;
    return 0;
}

} // namespace

int main(int argc, char *argv[])
{
    Options options;
    if (!parseOptions(argc, argv, options) || !validOptions(options)) {
        printUsage(argv[0]);
        return 64;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    YAML::Node config;
    try {
        config = YAML::LoadFile(options.config_path);
    } catch (const std::exception &error) {
        std::cerr << "H2_TEST_CONFIG_FAILED error=" << error.what()
                  << std::endl;
        return 65;
    }
    if (options.mode == Mode::Velocity &&
        !validateVelocityConfig(config, options)) {
        return 65;
    }

    std::shared_ptr<RobotHardwareInterface> robot;
    try {
        robot = RobotFactory::RobotAllocate(config);
    } catch (const std::exception &error) {
        std::cerr << "H2_TEST_FACTORY_FAILED error=" << error.what()
                  << std::endl;
        return 66;
    }
    const auto h2 = std::dynamic_pointer_cast<UnitreeH2>(robot);
    if (!h2) {
        std::cerr << "H2_TEST_FACTORY_TYPE_FAILED" << std::endl;
        return 66;
    }

    const int32_t init_ret = robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "H2_TEST_INIT_FAILED ret=" << init_ret << std::endl;
        return 67;
    }

    if (options.mode == Mode::ReadOnly) {
        std::cout << "H2_READ_ONLY_INIT_OK" << std::endl;
        return 0;
    }
    if (options.mode == Mode::GetterAudit) return runGetterAudit(h2);

    int fsm_before = -1;
    const int32_t fsm_before_ret = h2->readFsmId(fsm_before);
    std::cout << "H2_HAL_READY init_ret=" << init_ret
              << " fsm_ret=" << fsm_before_ret
              << " fsm_id=" << fsm_before << std::endl;
    if (fsm_before_ret != CMD_SUCCESS) return 67;

    if (options.mode == Mode::ZeroStop) {
        const int32_t stop_ret = stopRobot(robot);
        int fsm_after = -1;
        const int32_t after_ret = h2->readFsmId(fsm_after);
        std::cout << "H2_ZERO_STOP_RESULT stop_ret=" << stop_ret
                  << " fsm_after_ret=" << after_ret
                  << " fsm_after=" << fsm_after << std::endl;
        const bool passed = stop_ret == CMD_SUCCESS &&
                            after_ret == CMD_SUCCESS &&
                            fsm_after == fsm_before;
        if (passed) {
            std::cout << "H2_ZERO_STOP_RPC_OK fsm_id=" << fsm_after
                      << std::endl;
        }
        return passed ? 0 : 68;
    }

    if (options.mode == Mode::Action) {
        const int32_t action_ret = robot->writeActionCommand(options.action);
        std::cout << "H2_ACTION_RESULT action=" << options.action
                  << " ret=" << action_ret << std::endl;
        return action_ret == CMD_SUCCESS ? 0 : 68;
    }

    const int expected_count =
        (options.duration_ms + kCommandPeriodMs - 1) / kCommandPeriodMs;
    RobotVelocityCommand command{options.vx, options.vy, options.omega};
    int sent_count = 0;
    int32_t command_ret = CMD_SUCCESS;
    const auto start = std::chrono::steady_clock::now();
    for (int sequence = 0; sequence < expected_count && !stop_requested;
         ++sequence) {
        command_ret = robot->writeRobotVelocityCommand(command);
        ++sent_count;
        std::cout << "H2_HAL_VELOCITY seq=" << sent_count
                  << " ret=" << command_ret << std::endl;
        if (command_ret != CMD_SUCCESS) break;
        const auto next = start +
            std::chrono::milliseconds((sequence + 1) * kCommandPeriodMs);
        std::this_thread::sleep_until(next);
    }

    const int32_t stop_ret = stopRobot(robot);
    int fsm_after = -1;
    const int32_t fsm_after_ret = h2->readFsmId(fsm_after);
    std::cout << "H2_HAL_RESULT command_ret=" << command_ret
              << " stop_ret=" << stop_ret
              << " sent_count=" << sent_count
              << " expected_count=" << expected_count
              << " fsm_after_ret=" << fsm_after_ret
              << " fsm_after=" << fsm_after
              << " interrupted=" << (stop_requested ? 1 : 0) << std::endl;

    return command_ret == CMD_SUCCESS && stop_ret == CMD_SUCCESS &&
                   sent_count == expected_count &&
                   fsm_after_ret == CMD_SUCCESS &&
                   fsm_after == fsm_before && !stop_requested
               ? 0
               : 68;
}
