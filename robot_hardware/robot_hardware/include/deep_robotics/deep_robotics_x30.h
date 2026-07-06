#ifndef DEEP_ROBOTICS_X30_H
#define DEEP_ROBOTICS_X30_H

#include "robot_hardware_interface.h"

class DeepRoboticsX30 : public RobotHardwareInterface
{
public:
    DeepRoboticsX30() {}
    DeepRoboticsX30(YAML::Node config);
    ~DeepRoboticsX30() = default;

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;
};

#endif  