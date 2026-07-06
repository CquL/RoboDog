#ifndef ROBOT_HARDWARE_INTERFACE_H
#define ROBOT_HARDWARE_INTERFACE_H
#include <math.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "robot_hardware_error_code.h"
#include "robot_hardware_constant.h"
struct RobotVelocityCommand {
    double vx, vy, omega;
};

class RobotHardwareInterface
{
public:
    RobotHardwareInterface() {}
    RobotHardwareInterface(YAML::Node config) : config_(config) {}
    virtual ~RobotHardwareInterface() = default;
    //初始化机器人硬件
    virtual int32_t initRobotHardware() = 0;

    //写入机器人控制命令
    virtual int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) = 0;

    //动作指令
    virtual int32_t writeActionCommand(std::string action) = 0;
protected:
    YAML::Node config_;
 
};

#endif