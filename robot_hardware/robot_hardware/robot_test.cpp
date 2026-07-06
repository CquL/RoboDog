#include "robot_factory.h"
#include <chrono>   // 用于时间控制
#include <thread>   // 用于线程休眠
#include <iostream>
#include <cmath>

// 定义控制命令结构体（需与 robot_hardware_interface.h 保持一致）
// struct RobotVelocityCommand {
//     double vx, vy, omega;
// };

/**
 * @brief 精准控制机器狗移动指定距离
 * * @param robot 机器人硬件指针
 * @param vx 前向速度 (m/s)
 * @param vy 侧向速度 (m/s)
 * @param distance 目标移动距离 (m) (仅针对vx方向，简单示例)
 */

const int CMD_SLEEP_MS = 2; // 发送间隔（ms）= 2ms
const int STOP_CMD_SEND_TIMES = 20;     // 停止指令发送次数（原5次→20次）
const int STOP_CMD_INTERVAL_MS = 10;    // 停止指令间隔（ms）

void move_distance(std::shared_ptr<RobotHardwareInterface> robot, double vx, double distance) {
    if (std::abs(vx) < 0.05) {
        std::cerr << "[Error] 速度过小，无法移动 (Min: 0.05 m/s)" << std::endl;
        return;
    }

    // 1. 计算所需时间 (秒) = 距离 / 速度
    // 注意：这里取绝对值计算时间，速度方向由vx符号决定
    double duration_sec = std::abs(distance / vx);
    int duration_ms = static_cast<int>(duration_sec * 1000);

    std::cout << "[Info] 开始移动: 目标距离 " << distance << "m, 速度 " << vx 
              << "m/s, 预计耗时 " << duration_sec << "s()" << duration_ms << "ms)"<<  std::endl;

    //RobotVelocityCommand move_cmd = {vx, 0.0, 0.0};
    RobotVelocityCommand move_cmd = {0.0, vx, 0.0};
    
    // 2. 记录起始时间 (使用 steady_clock 保证单调递增，不受系统修改时间影响)
    auto start_time = std::chrono::steady_clock::now();
    int loop_count = 0;
    while (true) {
        // 计算已消耗时间
        auto current_time = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time
        ).count();

        // 调试打印（每50次循环打印一次，避免刷屏）
        if (loop_count % 50 == 0) {
            std::cout << "[Debug] 已耗时: " << elapsed_ms << "ms / " << duration_ms << "ms | 循环次数: " << loop_count << std::endl;
        }

        // 3. 检查是否达到目标时间
        if (elapsed_ms >= duration_ms) {
            std::cout << "[Info] 达到理论移动时间：" << elapsed_ms << "ms" << std::endl;
            break; 
        }

        // 4. 持续发送移动指令
        // 机器狗底层通常需要持续收到指令才能保持运动（看门狗机制）
        robot->writeRobotVelocityCommand(move_cmd);
        
        auto cmd_send_end = std::chrono::steady_clock::now();
        int cmd_cost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            cmd_send_end - current_time
        ).count();
        int sleep_ms = std::max(0, CMD_SLEEP_MS - cmd_cost_ms); // 补偿耗时，避免累计误差
        
        // 5. 控制发送频率 (500Hz = 2ms)
        // 过于频繁发送会占用CPU，太慢会导致机器狗动作不流畅
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        loop_count++;
    }

    // 6. 发送停止指令
    // 时间到，立刻发送零速度指令
    std::cout << "[Info] 发送停止指令（" << STOP_CMD_SEND_TIMES << "次）..." << std::endl;
    RobotVelocityCommand stop_cmd = {0.0, 0.0, 0.0};
    for(int i = 0; i < STOP_CMD_SEND_TIMES; ++i) {
        robot->writeRobotVelocityCommand(stop_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(STOP_CMD_INTERVAL_MS));
    }

    // 8. 移动后稳定1秒，避免惯性影响后续动作
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[Info] 移动完成，已稳定停止。" << std::endl;
}
int main(int argc, char *argv[])
{
    
    // 加载配置文件并初始化机器人硬件
    YAML::Node config = YAML::LoadFile("../config.yaml");
    std::shared_ptr<RobotHardwareInterface> robot = RobotFactory::RobotAllocate(config);
    int init_ret =  robot->initRobotHardware();
    if (init_ret != CMD_SUCCESS) {
        std::cerr << "[Error] 机器人初始化失败！" << std::endl;
        return -1;
    }
    std::cout << "[Info] 等待连接稳定 (3秒)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    
    // 任务1：向前distance 3.0 米，vx 0.5 m/s
    move_distance(robot, 0.5, 2.0);//vx=1.2,vy=,w=1.0,
    std::this_thread::sleep_for(std::chrono::seconds(2));
    move_distance(robot, -0.5, -2.0);

    std::cout << "[Info] 程序结束" << std::endl;
    return 0;
}
