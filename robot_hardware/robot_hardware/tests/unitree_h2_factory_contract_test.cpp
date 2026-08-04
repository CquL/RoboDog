// RobotFactory 的 H2 离线契约：验证 robot_model 到具体适配器的映射、
// 未初始化控制拒绝和未知型号拒绝；不调用 initRobotHardware()，
// 因此不会创建 DDS 通道或访问实机。
#include "robot_factory.h"
#include "unitree/unitree_h2.h"

#include <iostream>
#include <memory>
#include <stdexcept>

int main()
{
    // 使用最小配置构造 H2。两个许可开关显式关闭，防止本测试配置被误解为
    // 可执行实机运动的配置。
    YAML::Node config;
    config["robot_model"] = "unitree_h2";
    config["network_interface_card_name"] = "offline-test-interface";
    config["allow_motion_commands"] = false;
    config["allow_state_changing_actions"] = false;

    const auto robot = RobotFactory::RobotAllocate(config);
    // 工厂返回公共抽象指针；dynamic_pointer_cast 只用于测试实际分配类型。
    if (!robot || !std::dynamic_pointer_cast<UnitreeH2>(robot)) {
        std::cerr << "unitree_h2 was not allocated as UnitreeH2" << std::endl;
        return 1;
    }

    RobotVelocityCommand zero_command{0.0, 0.0, 0.0};
    // 未初始化对象必须拒绝速度命令，即使输入为零，证明上层不能跳过 init。
    if (robot->writeRobotVelocityCommand(zero_command) !=
        ERROR_ROBOT_HARDWARE_INIT) {
        std::cerr << "uninitialized H2 adapter accepted a velocity command"
                  << std::endl;
        return 2;
    }

    bool rejected_unknown_model = false;
    // 未知 robot_model 必须抛出明确配置错误，不能回退到任意机器人类型。
    try {
        YAML::Node invalid_config;
        invalid_config["robot_model"] = "unknown_robot";
        (void)RobotFactory::RobotAllocate(invalid_config);
    } catch (const std::runtime_error &) {
        rejected_unknown_model = true;
    }

    if (!rejected_unknown_model) {
        std::cerr << "factory did not reject an unknown robot_model" << std::endl;
        return 3;
    }

    std::cout << "UNITREE_H2_FACTORY_CONTRACT_OK" << std::endl;
    return 0;
}
