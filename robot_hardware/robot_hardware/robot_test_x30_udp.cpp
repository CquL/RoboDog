// X30 UDP 方向测试程序。
// 这个程序不是正式导航算法，而是用于安全验证 DeepRoboticsX30 的可配置 UDP 速度后端。
// 默认模式是 zero，只初始化并发送零速度；所有会让机器狗运动的测试都必须显式指定。

// 引入 X30 适配类。这里会间接拿到 RobotHardwareInterface、RobotVelocityCommand、
// ACTION_* 动作常量和返回码等定义。
#include "deep_robotics_x30.h"

// chrono 用于控制测试持续时间、发送周期和状态切换等待时间。
#include <chrono>
// iostream 用于打印测试过程和错误信息。
#include <iostream>
// memory 用于 std::shared_ptr，保持和上层统一接口调用方式一致。
#include <memory>
// string 用于模式名、说明文字和配置路径。
#include <string>
// thread 用于 sleep_for，控制发送频率和安全等待。
#include <thread>
// vector 用于保存所有方向测试项。
#include <vector>

// 匿名命名空间：这些常量和辅助函数只在本 cpp 文件内可见。
namespace {

// X30 接口规格书要求轴指令以 50Hz 下发，即每 20ms 调用一次。
constexpr int kCommandHz = 50;
// 每次预停止时发送零速度的次数。25 次 * 20ms = 约 0.5 秒。
constexpr int kWarmupStopCount = 25;
// 前进/后退安全测试速度，单位 m/s。数值故意很小，避免第一次测试冲得太远。
constexpr double kSafeTestVx = 0.05;
// 左右平移安全测试速度，单位 m/s。比 vx 更小，因为侧向方向更容易误判。
constexpr double kSafeTestVy = 0.03;
// 原地转向安全测试角速度，单位 rad/s。数值很小，只用于确认方向。
constexpr double kSafeTestOmega = 0.15;
// 1m 线性距离测试的目标距离，单位 m。
constexpr double kOneMeterDistance = 1.0;
// 1m 前进/后退测试速度，单位 m/s。0.10m/s 跑 10 秒，理论位移 1m。
constexpr double kDistanceTestVx = 0.10;
// 1m 左右平移测试速度，单位 m/s。0.10m/s 跑 10 秒，理论位移 1m。
constexpr double kDistanceTestVy = 0.10;
// 标定短脉冲前进/后退速度，单位 m/s。用于先量实际速度比例，不直接长距离运行。
constexpr double kCalibrationTestVx = 0.05;
// 标定短脉冲左右平移速度，单位 m/s。
constexpr double kCalibrationTestVy = 0.05;
// 标定短脉冲持续时间，单位 ms。1 秒足够观察比例，风险比 10 秒 1m 模式低很多。
constexpr int kCalibrationTestDurationMs = 1000;
// 每个方向实际发送非零速度的时间，单位 ms。
constexpr int kSafeTestDurationMs = 800;
// 发送 start/stop motion toggle 后等待状态稳定的时间，单位 ms。
constexpr int kMotionStateSettleMs = 1200;
// all_safe 模式中两个方向测试之间的停顿时间，单位 ms。
constexpr int kBetweenDirectionPauseMs = 1200;

// 单个方向测试的描述。
struct DirectionTest {
    // 命令行模式名，例如 forward、backward、yaw_left。
    std::string mode;
    // 打印给操作者看的说明。
    std::string description;
    // 实际发给 robot->writeRobotVelocityCommand 的统一速度命令。
    RobotVelocityCommand command;
    // 当前测试持续发送非零速度的时间。
    std::chrono::milliseconds duration;
};

// Protocol notes from "X30 ros+udp V1.0.4":
// - 0x21010130: X linear velocity axis. Positive means forward, negative backward.
// - 0x21010131: Y linear velocity axis. Spec axis positive means right, negative left.
// - 0x21010135: yaw axis. Spec axis positive means right turn, negative left turn.
// - 0x21010201: start/stop motion toggle, force-stand <-> stepping.
// udp_axis exercises this legacy joystick path. udp_navigation reproduces the
// factory ROS /cmd_vel transport, while udp_physical reproduces /cmd_vel_corrected.
//
// 中文说明：
// 1. 规格书里的 0x21 是“手柄/遥控终端”发送源前缀。
// 2. 我们 config.yaml 里 motion_command_source: "navigation" 时，实际会改成 0x31 前缀。
// 3. 所以下面注释里的 0x21010130/31/35 是规格书默认写法，程序实际发送可能是 0x31010130/31/35。
// 4. velocity_backend 决定实际发送轴值、物理速度，还是经 105 安全层的导航速度包。

// 根据目标距离和速度计算理论运行时间。
// 例：1m / 0.10m/s = 10s = 10000ms。
std::chrono::milliseconds durationFromDistance(double distance_m, double speed_mps)
{
    if (distance_m <= 0.0 || speed_mps <= 0.0) {
        return std::chrono::milliseconds(0);
    }

    return std::chrono::milliseconds(
        static_cast<int>((distance_m / speed_mps) * 1000.0)
    );
}

// 返回所有可测试的方向列表。
// 用 static 是为了只构造一次，后面每次调用都复用同一份表。
const std::vector<DirectionTest> &directionTests()
{
    static const std::vector<DirectionTest> tests = {
        // 前进：vx 为正，vy 和 omega 为 0。
        {"forward", "small forward translation: vx > 0", {kSafeTestVx, 0.0, 0.0}, std::chrono::milliseconds(kSafeTestDurationMs)},
        // 后退：vx 为负，vy 和 omega 为 0。
        {"backward", "small backward translation: vx < 0", {-kSafeTestVx, 0.0, 0.0}, std::chrono::milliseconds(kSafeTestDurationMs)},
        // 左移：当前统一接口约定里，用 vy > 0 表示左移测试。
        // DeepRoboticsX30 内部会根据 invert_vy_axis 配置转换成 X30 轴值。
        {"left", "small left translation: vy > 0 in RobotVelocityCommand", {0.0, kSafeTestVy, 0.0}, std::chrono::milliseconds(kSafeTestDurationMs)},
        // 右移：当前统一接口约定里，用 vy < 0 表示右移测试。
        {"right", "small right translation: vy < 0 in RobotVelocityCommand", {0.0, -kSafeTestVy, 0.0}, std::chrono::milliseconds(kSafeTestDurationMs)},
        // 左转：统一接口采用 ROS Twist 约定，omega > 0 表示逆时针/左转。
        {"yaw_left", "small left yaw test: omega > 0", {0.0, 0.0, kSafeTestOmega}, std::chrono::milliseconds(kSafeTestDurationMs)},
        // 右转：统一接口采用 ROS Twist 约定，omega < 0 表示顺时针/右转。
        {"yaw_right", "small right yaw test: omega < 0", {0.0, 0.0, -kSafeTestOmega}, std::chrono::milliseconds(kSafeTestDurationMs)},
    };

    // 返回静态测试表，供 list、all_safe 和单方向测试复用。
    return tests;
}

// 返回 1m 线性距离测试列表。
// 这组测试只做前后左右平移，不包含 yaw，因为 yaw 是角度不是距离。
const std::vector<DirectionTest> &distanceTests()
{
    static const std::vector<DirectionTest> tests = {
        // 理论前进 1m：0.10m/s 持续 10s。
        {"forward_1m", "theoretical 1m forward translation", {kDistanceTestVx, 0.0, 0.0}, durationFromDistance(kOneMeterDistance, kDistanceTestVx)},
        // 理论后退 1m：-0.10m/s 持续 10s。
        {"backward_1m", "theoretical 1m backward translation", {-kDistanceTestVx, 0.0, 0.0}, durationFromDistance(kOneMeterDistance, kDistanceTestVx)},
        // 理论左移 1m：0.10m/s 持续 10s。
        {"left_1m", "theoretical 1m left translation", {0.0, kDistanceTestVy, 0.0}, durationFromDistance(kOneMeterDistance, kDistanceTestVy)},
        // 理论右移 1m：-0.10m/s 持续 10s。
        {"right_1m", "theoretical 1m right translation", {0.0, -kDistanceTestVy, 0.0}, durationFromDistance(kOneMeterDistance, kDistanceTestVy)},
    };

    return tests;
}

// 返回短脉冲标定测试列表。
// 如果 1m 开环距离明显不准，先用这些模式量 1 秒实际位移，再反推速度比例。
const std::vector<DirectionTest> &calibrationTests()
{
    static const std::vector<DirectionTest> tests = {
        // 前进标定：0.05m/s 持续 1s，理论位移 0.05m。
        {"forward_calib", "1s forward calibration pulse", {kCalibrationTestVx, 0.0, 0.0}, std::chrono::milliseconds(kCalibrationTestDurationMs)},
        // 后退标定：-0.05m/s 持续 1s，理论位移 0.05m。
        {"backward_calib", "1s backward calibration pulse", {-kCalibrationTestVx, 0.0, 0.0}, std::chrono::milliseconds(kCalibrationTestDurationMs)},
        // 左移标定：0.05m/s 持续 1s，理论位移 0.05m。
        {"left_calib", "1s left calibration pulse", {0.0, kCalibrationTestVy, 0.0}, std::chrono::milliseconds(kCalibrationTestDurationMs)},
        // 右移标定：-0.05m/s 持续 1s，理论位移 0.05m。
        {"right_calib", "1s right calibration pulse", {0.0, -kCalibrationTestVy, 0.0}, std::chrono::milliseconds(kCalibrationTestDurationMs)},
    };

    return tests;
}

// 把历史命令或别名转换成正式模式名。
// 这样旧命令 move 还可以继续使用，不影响之前的上机习惯。
std::string canonicalMode(const std::string &mode)
{
    // 旧版 move 模式等价于现在的 forward。
    if (mode == "move") {
        return "forward";
    }

    // turn_left 是 yaw_left 的可读别名。
    if (mode == "turn_left") {
        return "yaw_left";
    }

    // turn_right 是 yaw_right 的可读别名。
    if (mode == "turn_right") {
        return "yaw_right";
    }

    // 没有匹配别名时，原样返回。
    return mode;
}

// start/stop motion 是摇杆轴值链路的状态切换；原厂物理速度和导航速度
// 后端由 APP/遥控器控制模式，不应在每轮速度测试前后盲目 toggle。
bool backendUsesMotionToggle(const YAML::Node &config)
{
    const std::string backend = config["velocity_backend"]
        ? config["velocity_backend"].as<std::string>()
        : "udp_axis";
    return backend == "udp_axis";
}

// 根据模式名查找方向测试项。
// 找到时返回指向 DirectionTest 的指针，找不到时返回 nullptr。
const DirectionTest *findDirectionTest(const std::string &mode)
{
    // 先把别名转换成正式模式名。
    const std::string canonical = canonicalMode(mode);

    // 遍历所有方向测试。
    for (const auto &test : directionTests()) {
        // 如果模式名一致，就返回这个测试项地址。
        if (test.mode == canonical) {
            return &test;
        }
    }

    // 再查找短脉冲标定模式。
    for (const auto &test : calibrationTests()) {
        if (test.mode == canonical) {
            return &test;
        }
    }

    // 再查找 1m 距离测试模式。
    for (const auto &test : distanceTests()) {
        if (test.mode == canonical) {
            return &test;
        }
    }

    // 没找到说明不是有效方向模式。
    return nullptr;
}

// 判断第一个命令行参数是不是“模式名”而不是“配置文件路径”。
// 支持两种写法：
//   ./robot_test_x30_udp ../config.yaml forward
//   ./robot_test_x30_udp forward
bool isModeOnlyArgument(const std::string &arg)
{
    // 先处理别名。
    const std::string canonical = canonicalMode(arg);

    // zero/list/help/all_safe 是特殊模式；方向模式通过 findDirectionTest 判断。
    return arg == "zero" || arg == "list" || arg == "help" || arg == "--help" ||
           arg == "all_safe" || arg == "all_calib" || arg == "all_1m" ||
           findDirectionTest(canonical) != nullptr;
}

// 打印使用说明。
// list/help/--help 模式会调用这个函数，不会初始化机器人，也不会发送 UDP。
void printUsage(const char *program)
{
    // 打印基本命令格式。
    std::cout << "Usage:\n"
              << "  " << program << " [config.yaml] zero\n"
              << "  " << program << " [config.yaml] forward|backward|left|right|yaw_left|yaw_right\n"
              << "  " << program << " [config.yaml] forward_calib|backward_calib|left_calib|right_calib\n"
              << "  " << program << " [config.yaml] forward_1m|backward_1m|left_1m|right_1m\n"
              << "  " << program << " [config.yaml] all_safe\n"
              << "  " << program << " [config.yaml] all_calib\n"
              << "  " << program << " [config.yaml] all_1m\n"
              << "  " << program << " [config.yaml] list\n\n"
              // 打印当前安全速度参数，便于上机前确认。
              << "Safety defaults:\n"
              << "  vx=" << kSafeTestVx << " m/s, vy=" << kSafeTestVy
              << " m/s, omega=" << kSafeTestOmega << " rad/s, duration="
              << kSafeTestDurationMs << " ms\n\n"
              << "1m distance defaults:\n"
              << "  distance=" << kOneMeterDistance << " m, vx=" << kDistanceTestVx
              << " m/s, vy=" << kDistanceTestVy << " m/s, duration="
              << durationFromDistance(kOneMeterDistance, kDistanceTestVx).count()
              << " ms\n\n"
              << "Calibration pulse defaults:\n"
              << "  vx=" << kCalibrationTestVx << " m/s, vy=" << kCalibrationTestVy
              << " m/s, duration=" << kCalibrationTestDurationMs << " ms\n\n"
              // 打印兼容别名。
              << "Aliases:\n"
              << "  move -> forward\n"
              << "  turn_left -> yaw_left\n"
              << "  turn_right -> yaw_right\n\n"
              // 打印方向测试列表。
              << "Available direction tests:\n";

    // 把 directionTests() 中的每个短方向测试项打印出来。
    for (const auto &test : directionTests()) {
        std::cout << "  " << test.mode << " : " << test.description << "\n";
    }

    // 打印短脉冲标定测试项。
    std::cout << "\nAvailable calibration pulse tests:\n";
    for (const auto &test : calibrationTests()) {
        std::cout << "  " << test.mode << " : " << test.description
                  << " (" << test.duration.count() << " ms)\n";
    }

    // 打印 1m 距离测试项。
    std::cout << "\nAvailable 1m distance tests:\n";
    for (const auto &test : distanceTests()) {
        std::cout << "  " << test.mode << " : " << test.description
                  << " (" << test.duration.count() << " ms)\n";
    }

    // 额外提醒操作者先跑 zero，方向测试必须一个一个来。
    std::cout << "\nRun zero first. Run one direction at a time with the remote controller ready.\n";
}

// 在指定 duration 时间内，以 kCommandHz 频率持续发送同一个速度命令。
// 机器狗底层通常需要持续收到速度命令，不能只发一次。
bool publishFor(std::shared_ptr<RobotHardwareInterface> robot,
                RobotVelocityCommand cmd,
                std::chrono::milliseconds duration)
{
    // 计算发送周期。50Hz -> 1000/50 = 20ms。
    auto period = std::chrono::milliseconds(1000 / kCommandHz);

    // 计算测试结束时间点。
    auto end_time = std::chrono::steady_clock::now() + duration;

    // 在结束时间到来前持续发送。
    while (std::chrono::steady_clock::now() < end_time) {
        // 通过统一接口写入速度命令。
        // X30 底层会按 velocity_backend 转成对应的 UDP 报文。
        int32_t ret = robot->writeRobotVelocityCommand(cmd);

        // 如果底层返回错误，打印错误并终止本方向测试。
        if (ret != CMD_SUCCESS) {
            std::cerr << "[robot_test_x30_udp] Velocity command failed, ret="
                      << ret << std::endl;
            return false;
        }

        // 等待下一个发送周期。
        std::this_thread::sleep_for(period);
    }

    // 持续发送阶段正常结束。
    return true;
}

// 连续发送零速度。
// 这个函数用于测试前、测试后和异常时兜底停止。
void sendStop(std::shared_ptr<RobotHardwareInterface> robot)
{
    // 构造零速度命令。
    RobotVelocityCommand stop_cmd{0.0, 0.0, 0.0};

    // 连续发送多次，避免单个 UDP 包丢失导致停止不可靠。
    for (int i = 0; i < kWarmupStopCount; ++i) {
        // 对 X30 来说，这会通过当前 velocity_backend 发送零速度。
        robot->writeRobotVelocityCommand(stop_cmd);

        // 两次零速度之间间隔 20ms，与规格书的 50Hz 一致。
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// 发送 X30 start/stop motion toggle。
// 规格书含义：力控站立 <-> 踏步状态之间轮流切换。
bool toggleStartStopMotion(std::shared_ptr<RobotHardwareInterface> robot,
                           const std::string &expected_transition)
{
    // 打印我们期望这一次 toggle 发生的状态变化。
    std::cout << "[robot_test_x30_udp] start/stop motion toggle. Expected: "
              << expected_transition << std::endl;

    // 通过统一动作接口发送 ACTION_START_STOP_MOTION。
    // X30 底层会转换成 0x31010201 或 0x21010201。
    int32_t ret = robot->writeActionCommand(ACTION_START_STOP_MOTION);

    // 如果发送失败，返回 false。
    if (ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_udp] start/stop motion toggle failed, ret="
                  << ret << std::endl;
        return false;
    }

    // 等待状态机稳定。这个等待只在测试程序里有，底层接口没有这个延时。
    std::this_thread::sleep_for(std::chrono::milliseconds(kMotionStateSettleMs));

    // toggle 发送和等待都完成。
    return true;
}

// 执行单个方向测试。
// 安全流程：
//   1. 先发零速度
//   2. 切到踏步/可响应速度状态
//   3. 短时间发送某个方向速度
//   4. 发零速度停止
//   5. 切回力控站立
//   6. 再发零速度兜底
bool runDirectionTest(std::shared_ptr<RobotHardwareInterface> robot,
                      const DirectionTest &test,
                      bool use_motion_toggle)
{
    // 打印当前方向测试名称和说明。
    std::cout << "[robot_test_x30_udp] Direction test: " << test.mode
              << " | " << test.description << std::endl;

    // 打印即将发送的统一速度命令。
    std::cout << "[robot_test_x30_udp] Command: vx=" << test.command.vx
              << ", vy=" << test.command.vy
              << ", omega=" << test.command.omega
              << ", duration_ms=" << test.duration.count() << std::endl;

    // 测试前先发零速度，确保上一轮残留速度被清掉。
    sendStop(robot);

    if (use_motion_toggle) {
        // 只有 udp_axis 使用摇杆 start/stop motion 状态切换。
        if (!toggleStartStopMotion(robot, "force stand -> stepping")) {
            sendStop(robot);
            return false;
        }
    } else {
        std::cout << "[robot_test_x30_udp] Motion toggle skipped for factory "
                  << "velocity backend; set robot mode with the APP/remote."
                  << std::endl;
    }

    // 在安全时长内持续发送这个方向的速度命令。
    bool ok = publishFor(
        robot,
        test.command,
        test.duration
    );

    // 非零速度发送结束，准备停止。
    std::cout << "[robot_test_x30_udp] Stopping after " << test.mode << "."
              << std::endl;

    // 先连续发送零速度，尽快让速度归零。
    sendStop(robot);

    // 再调用统一动作 stop_move。X30 中该动作内部也会连续发零速度。
    robot->writeActionCommand(ACTION_STOP_MOVE);

    if (use_motion_toggle) {
        // udp_axis 测试结束后切回力控站立。
        if (!toggleStartStopMotion(robot, "stepping -> force stand")) {
            sendStop(robot);
            return false;
        }
    }

    // 最后再发一次零速度，作为测试结束兜底。
    sendStop(robot);

    // 返回 publishFor 的结果，表示方向速度发送阶段是否成功。
    return ok;
}

} // namespace

// 程序入口。
// 支持：
//   ./robot_test_x30_udp ../config.yaml zero
//   ./robot_test_x30_udp ../config.yaml forward
//   ./robot_test_x30_udp forward
//   ./robot_test_x30_udp list
int main(int argc, char *argv[])
{
    // 默认配置文件路径。程序通常从 build_x30 目录运行，所以 ../config.yaml 指向源码目录配置。
    std::string config_path = "../config.yaml";

    // 默认模式是 zero，保证不带参数运行时不会让机器狗运动。
    std::string mode = "zero";

    // 如果用户提供了第一个参数，需要判断它是模式名还是配置路径。
    if (argc > 1) {
        // 保存第一个参数。
        std::string first_arg = argv[1];

        // 如果第一个参数本身就是模式名，就使用默认配置路径。
        if (isModeOnlyArgument(first_arg)) {
            mode = first_arg;
        } else {
            // 否则把第一个参数当作配置文件路径。
            config_path = first_arg;
        }
    }

    // 如果用户提供了第二个参数，则第二个参数明确作为模式名。
    if (argc > 2) {
        mode = argv[2];
    }

    // list/help/--help 只打印说明，不初始化硬件，也不发 UDP。
    if (mode == "list" || mode == "help" || mode == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    // 读取 YAML 配置。
    YAML::Node config = YAML::LoadFile(config_path);

    // 这个测试程序只允许用于 deep_robotics_x30，避免误拿其它机器人配置运行。
    if (config["robot_model"].as<std::string>() != "deep_robotics_x30") {
        std::cerr << "[robot_test_x30_udp] config robot_model must be deep_robotics_x30."
                  << std::endl;
        return 1;
    }

    const bool use_motion_toggle = backendUsesMotionToggle(config);

    // 创建机器人对象。这里直接创建 DeepRoboticsX30，便于测试 X30 UDP 后端。
    // 正式上层算法也可以通过 RobotFactory 创建 RobotHardwareInterface 指针。
    std::shared_ptr<RobotHardwareInterface> robot =
        std::make_shared<DeepRoboticsX30>(config);

    // 初始化硬件通信。X30 里会创建 UDP socket、设置目标地址、发心跳和确认连接。
    int32_t init_ret = robot->initRobotHardware();

    // 初始化失败就退出，后面不再发速度。
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "[robot_test_x30_udp] Robot init failed, ret="
                  << init_ret << std::endl;
        return 1;
    }

    // 初始化成功后，先发一轮零速度。
    std::cout << "[robot_test_x30_udp] Initialized. Sending zero velocity first..."
              << std::endl;
    sendStop(robot);

    // zero 模式：只初始化和发零速度，不执行任何运动方向测试。
    if (mode == "zero") {
        std::cout << "[robot_test_x30_udp] Zero-only mode finished. Robot should not move."
                  << std::endl;

        // 再通过动作接口发 stop_move，确保停止命令链路也被验证。
        robot->writeActionCommand(ACTION_STOP_MOVE);
        return 0;
    }

    // all_safe 模式：按顺序跑所有方向测试。
    // 注意：这个模式会让机器狗依次前后左右和转向，必须在单方向都确认后再使用。
    if (mode == "all_safe") {
        std::cout << "[robot_test_x30_udp] ALL_SAFE MODE: this will run every "
                  << "direction test one by one. Keep a wide clear area and "
                  << "hold the remote controller." << std::endl;

        // 遍历所有方向测试。
        for (const auto &test : directionTests()) {
            // 如果某个方向失败，立即发零速度和 stop_move，然后退出。
            if (!runDirectionTest(robot, test, use_motion_toggle)) {
                sendStop(robot);
                robot->writeActionCommand(ACTION_STOP_MOVE);
                return 1;
            }

            // 两个方向之间停顿一下，方便观察，也避免连续动作太密。
            std::this_thread::sleep_for(std::chrono::milliseconds(kBetweenDirectionPauseMs));
        }

        // 所有方向执行完成。
        std::cout << "[robot_test_x30_udp] all_safe finished." << std::endl;
        return 0;
    }

    // all_calib 模式：按顺序跑所有短脉冲标定测试。
    // 注意：如果 1m 测试明显超距，先用这个模式标定实际速度比例。
    if (mode == "all_calib") {
        std::cout << "[robot_test_x30_udp] ALL_CALIB MODE: this will run every "
                  << "1s calibration pulse one by one. Keep a wide clear area and "
                  << "hold the remote controller." << std::endl;

        for (const auto &test : calibrationTests()) {
            if (!runDirectionTest(robot, test, use_motion_toggle)) {
                sendStop(robot);
                robot->writeActionCommand(ACTION_STOP_MOVE);
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kBetweenDirectionPauseMs));
        }

        std::cout << "[robot_test_x30_udp] all_calib finished." << std::endl;
        return 0;
    }

    // all_1m 模式：按顺序跑所有 1m 线性距离测试。
    // 注意：这个模式每个方向理论运行 1m，必须在短方向测试全部确认后再使用。
    if (mode == "all_1m") {
        std::cout << "[robot_test_x30_udp] ALL_1M MODE: this will run every "
                  << "1m linear distance test one by one. Keep a wide clear area and "
                  << "hold the remote controller." << std::endl;

        // 遍历所有 1m 距离测试。
        for (const auto &test : distanceTests()) {
            if (!runDirectionTest(robot, test, use_motion_toggle)) {
                sendStop(robot);
                robot->writeActionCommand(ACTION_STOP_MOVE);
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kBetweenDirectionPauseMs));
        }

        std::cout << "[robot_test_x30_udp] all_1m finished." << std::endl;
        return 0;
    }

    // 单方向模式：根据 mode 查找对应 DirectionTest。
    const DirectionTest *direction_test = findDirectionTest(mode);

    // 如果找不到，说明用户输入了未知模式。
    if (direction_test == nullptr) {
        std::cerr << "[robot_test_x30_udp] Unknown mode: " << mode
                  << "." << std::endl;

        // 打印帮助，便于现场直接看到可用模式。
        printUsage(argv[0]);

        // 退出前发 stop_move 兜底。
        robot->writeActionCommand(ACTION_STOP_MOVE);
        return 1;
    }

    // 执行单方向测试。
    if (!runDirectionTest(robot, *direction_test, use_motion_toggle)) {
        // 如果失败，退出前再发 stop_move。
        robot->writeActionCommand(ACTION_STOP_MOVE);
        return 1;
    }

    // 单方向测试正常结束。
    std::cout << "[robot_test_x30_udp] Done." << std::endl;

    // 返回 0 表示程序成功。
    return 0;
}
