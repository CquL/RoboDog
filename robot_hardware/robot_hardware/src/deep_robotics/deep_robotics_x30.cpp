#include "deep_robotics_x30.h"

DeepRoboticsX30::DeepRoboticsX30(YAML::Node config) : RobotHardwareInterface(config) {
    
}

int32_t DeepRoboticsX30::initRobotHardware()
{
    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::writeRobotVelocityCommand(RobotVelocityCommand &cmd)
{
    return CMD_SUCCESS;
}

int32_t DeepRoboticsX30::writeActionCommand(std::string action)
{
    return CMD_SUCCESS;
}