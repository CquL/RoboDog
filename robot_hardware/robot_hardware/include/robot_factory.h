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
#include "unitree/unitree_dog.h"
#include "unitree/unitree_h2.h"
#include "zsibot/zsibot_zsl_one.h"
#include "deep_robotics/deep_robotics_x30.h"
class RobotFactory
{
public:
    static std::shared_ptr<RobotHardwareInterface> RobotAllocate(YAML::Node node){
        std::string robot_type = node["robot_model"].as<std::string>();
        if(robot_type == "unitree_dog"){
            return std::make_shared<UnitreeDog>(node);
        }else if(robot_type == "unitree_h2"){
            return std::make_shared<UnitreeH2>(node);
        }else if(robot_type == "zsibot_zsl_one"){
            return std::make_shared<ZsibotZslOne>(node);
        }else if(robot_type == "deep_robotics_x30"){
            return std::make_shared<DeepRoboticsX30>(node);
        }else{
            throw std::runtime_error("Invalid robot type: " + robot_type);
        }
    }
};

#endif
