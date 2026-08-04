#include "deep_robotics/deep_robotics_x30.h"
#include "robot_factory.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

// 在不初始化 UDP 的情况下验证编译期 X30-only RobotFactory 边界。
// 测试同时覆盖 X30 正常分配和对已排除厂家型号的拒绝。
int main()
{
    // 支持的型号必须仍可通过通用接口获取。
    YAML::Node x30_config;
    x30_config["robot_model"] = "deep_robotics_x30";
    const std::shared_ptr<RobotHardwareInterface> robot =
        RobotFactory::RobotAllocate(x30_config);
    if (dynamic_cast<DeepRoboticsX30 *>(robot.get()) == nullptr) {
        std::cerr << "X30-only factory did not allocate DeepRoboticsX30."
                  << std::endl;
        return 1;
    }

    // X30-only 镜像必须明确失败，不能暴露构建时已省略厂家依赖的型号。
    YAML::Node unsupported_config;
    unsupported_config["robot_model"] = "unitree_h2";
    try {
        (void)RobotFactory::RobotAllocate(unsupported_config);
    } catch (const std::runtime_error &error) {
        const std::string message = error.what();
        if (message.find("unavailable in the X30-only build") !=
            std::string::npos) {
            std::cout << "X30-only RobotFactory contract passed."
                      << std::endl;
            return 0;
        }
    }

    std::cerr << "X30-only factory accepted an unavailable robot model."
              << std::endl;
    return 1;
}
