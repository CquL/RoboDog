// H2 分阶段实机测试命令行入口。
//
// 本程序不是生产控制器；它用于逐项验证 RobotFactory ->
// RobotHardwareInterface（动态对象为 UnitreeH2）-> SDK2 的控制链。只读查询不接受 --execute，任何可能
// 写入机器人的停止、非零速度或状态动作都必须显式提供 --execute。
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

// 速度流固定为 20 Hz；单次 CLI 最短 100 ms、最长 3000 ms。这里限制的是
// 测试程序持续发送的总时长，不是 SDK SetVelocity 的单条 duration 参数。
constexpr int kCommandPeriodMs = 50;
constexpr int kMinStreamMs = 100;
constexpr int kMaxStreamMs = 3000;

// SIGINT/SIGTERM 只设置原子标志；真正的 StopMove 在主线程退出发送循环后执行。
std::atomic<bool> stop_requested{false};

// 一次进程只允许选择一种模式，避免例如 getter 与 velocity 混在同一命令中，
// 使实机日志无法判断到底执行了哪类操作。
enum class Mode {
    None,
    ReadOnly,
    GetterAudit,
    ZeroStop,
    Velocity,
    Action,
};

// 完整保存解析后的命令行。速度单位为 m/s、角速度单位为 rad/s、时长为 ms。
// execute 是所有写操作的显式人工确认标志。
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

// 用法文本同时定义 CLI 契约；参数校验失败统一返回 main() 中的退出码 64。
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
    // 只允许从 None 选择一次，第二个模式参数会令整条命令无效。
    if (options.mode != Mode::None) return false;
    options.mode = mode;
    return true;
}

bool parseDouble(const char *text, double &value)
{
    // 必须消费完整字符串且结果有限，拒绝 "1.0abc"、NaN 和 Infinity。
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
    // 同样要求完整消费文本，避免把带后缀的时长误当作合法整数。
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
    // 手工解析保持二进制无额外 CLI 依赖；任何重复、缺值或未知参数都失败。
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
    // 只读模式必须没有 --execute；所有写模式必须有 --execute。Velocity 还要求
    // 至少一个非零轴及受限总时长，避免把零速度误记为非零运动验证。
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
    // 只用于测试程序的预检；真正的配置解析和硬上限校验仍由 UnitreeH2 完成。
    return config[key] ? config[key].as<T>() : default_value;
}

bool validateVelocityConfig(const YAML::Node &config, const Options &options)
{
    // 在创建 SDK 客户端前先检查 motion YAML，防止误拿只读配置执行非零速度。
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
        // 底层 HAL 会 clamp，但实机测试拒绝“请求值与实际值不同”的情况，确保
        // 命令行、日志与最终发送给 SDK 的数值一致。
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
    // 停止路径同时调用动作接口 StopMove 和统一速度接口的全零命令，用日志分别
    // 暴露两个返回值；最终优先返回 StopMove 错误。
    const int32_t action_ret = robot->writeActionCommand(ACTION_STOP_MOVE);
    RobotVelocityCommand zero{0.0, 0.0, 0.0};
    const int32_t zero_ret = robot->writeRobotVelocityCommand(zero);
    std::cout << "H2_HAL_STOP action_ret=" << action_ret
              << " zero_ret=" << zero_ret << std::endl;
    return action_ret != CMD_SUCCESS ? action_ret : zero_ret;
}

int runGetterAudit(const std::shared_ptr<UnitreeH2> &h2)
{
    // Getter audit 只执行 SDK2 状态读取，不发送速度或动作。fsm_mode 当前只作
    // 观测记录，不在本程序中推导其固件语义。
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
        // 原厂可能返回 ID 数量多于名称数量，因此输出时做边界检查。
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
    // 第一道门：命令行模式、--execute 和数值范围必须自洽。
    Options options;
    if (!parseOptions(argc, argv, options) || !validOptions(options)) {
        printUsage(argv[0]);
        return 64;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // 第二道门：YAML 必须可解析；velocity 模式还要提前验证运动许可与限幅。
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
        // 工厂依据 robot_model: unitree_h2 返回 UnitreeH2，但后续控制仍通过
        // RobotHardwareInterface 调用，以验证上层抽象协议确实可复用。
        robot = RobotFactory::RobotAllocate(config);
    } catch (const std::exception &error) {
        std::cerr << "H2_TEST_FACTORY_FAILED error=" << error.what()
                  << std::endl;
        return 66;
    }
    const auto h2 = std::dynamic_pointer_cast<UnitreeH2>(robot);
    if (!h2) {
        // H2 getter 不属于公共三接口，因此诊断模式额外核对工厂实际类型。
        std::cerr << "H2_TEST_FACTORY_TYPE_FAILED" << std::endl;
        return 66;
    }

    const int32_t init_ret = robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "H2_TEST_INIT_FAILED ret=" << init_ret << std::endl;
        return 67;
    }

    if (options.mode == Mode::ReadOnly) {
        // 只验证 DDS/SDK 初始化和可选 FSM 读取，不执行任何控制 API。
        std::cout << "H2_READ_ONLY_INIT_OK" << std::endl;
        return 0;
    }
    if (options.mode == Mode::GetterAudit) return runGetterAudit(h2);

    int fsm_before = -1;
    // 所有写模式执行前记录 FSM；ZeroStop 和 Velocity 结束后还会再次读取并
    // 检查一致性。Action 可能主动改变状态，因此只检查动作前 getter 可用。
    const int32_t fsm_before_ret = h2->readFsmId(fsm_before);
    std::cout << "H2_HAL_READY init_ret=" << init_ret
              << " fsm_ret=" << fsm_before_ret
              << " fsm_id=" << fsm_before << std::endl;
    if (fsm_before_ret != CMD_SUCCESS) return 67;

    if (options.mode == Mode::ZeroStop) {
        // 零速门禁会实际调用 StopMove/零速度，但不会发送非零速度；用于先验证
        // 高层停止链路和 FSM 稳定性；它不等同于硬件急停。
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
        // 状态动作由 HAL 白名单、allow_state_changing_actions 和 SDK 返回值共同
        // 决定是否成功；CLI 不替调用方自动追加其他动作。
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
    // 以绝对时间点 sleep_until 保持 20 Hz，避免每次 RPC 耗时累计漂移。每帧
    // 都走 writeRobotVelocityCommand，因此每帧都会受到 HAL 限幅和 FSM 门禁。
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

    // 无论发送完成、失败还是收到信号，离开循环后都执行停止并读取结束 FSM。
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

    // 只有全部计划帧、停止路径、FSM 前后一致且未被信号中断才返回成功。
    return command_ret == CMD_SUCCESS && stop_ret == CMD_SUCCESS &&
                   sent_count == expected_count &&
                   fsm_after_ret == CMD_SUCCESS &&
                   fsm_after == fsm_before && !stop_requested
               ? 0
               : 68;
}
