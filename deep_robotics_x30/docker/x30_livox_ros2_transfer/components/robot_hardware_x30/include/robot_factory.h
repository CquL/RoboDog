#ifndef ROBOT_FACTORY_H
#define ROBOT_FACTORY_H
#include <stdint.h>
#include <mutex>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "robot_hardware_interface.h"
//#include "virtual_robot/virtual_robot.h"
#ifndef ROBOT_HARDWARE_X30_ONLY
// 完整构建包含 H2 适配器及其 SDK2 类型；X30-only 构建不解析这些厂商头文件。
#include "unitree/unitree_dog.h"
#include "unitree/unitree_h2.h"
#include "zsibot/zsibot_zsl_one.h"
#endif
#include "deep_robotics/deep_robotics_x30.h"

// 运行时选择具体机器人 HAL。X30-only 构建主动排除 Unitree 和 Zsibot 头文件，
// 避免可迁移 X30 镜像携带无用厂商依赖。
class RobotFactory
{
public:
    // 读取 robot_model，并通过通用接口返回匹配实现。X30-only 构建明确拒绝
    // 不可用型号，避免静默构造链接不完整的厂商后端。
    static std::shared_ptr<RobotHardwareInterface> RobotAllocate(YAML::Node node){
        std::string robot_type = node["robot_model"].as<std::string>();
#ifdef ROBOT_HARDWARE_X30_ONLY
        if(robot_type == "deep_robotics_x30"){
            return std::make_shared<DeepRoboticsX30>(node);
        }else{
            throw std::runtime_error(
                "Robot type is unavailable in the X30-only build: " +
                robot_type);
        }
#else
        if(robot_type == "unitree_dog"){
            return std::make_shared<UnitreeDog>(node);
        }else if(robot_type == "unitree_h2"){
            // H2 与其他机器人一样只增加一个独立分支。完整 YAML 原样传入
            // UnitreeH2，由适配器读取 DDS、限幅、FSM 和安全许可参数。
            return std::make_shared<UnitreeH2>(node);
        }else if(robot_type == "zsibot_zsl_one"){
            return std::make_shared<ZsibotZslOne>(node);
        }else if(robot_type == "deep_robotics_x30"){
            return std::make_shared<DeepRoboticsX30>(node);
        }else{
            throw std::runtime_error("Invalid robot type: " + robot_type);
        }
#endif
    }
};

#endif
