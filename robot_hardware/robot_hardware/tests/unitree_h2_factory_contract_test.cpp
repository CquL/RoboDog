#include "robot_factory.h"
#include "unitree/unitree_h2.h"

#include <iostream>
#include <memory>
#include <stdexcept>

int main()
{
    YAML::Node config;
    config["robot_model"] = "unitree_h2";
    config["network_interface_card_name"] = "offline-test-interface";
    config["allow_motion_commands"] = false;
    config["allow_state_changing_actions"] = false;

    const auto robot = RobotFactory::RobotAllocate(config);
    if (!robot || !std::dynamic_pointer_cast<UnitreeH2>(robot)) {
        std::cerr << "unitree_h2 was not allocated as UnitreeH2" << std::endl;
        return 1;
    }

    RobotVelocityCommand zero_command{0.0, 0.0, 0.0};
    if (robot->writeRobotVelocityCommand(zero_command) !=
        ERROR_ROBOT_HARDWARE_INIT) {
        std::cerr << "uninitialized H2 adapter accepted a velocity command"
                  << std::endl;
        return 2;
    }

    bool rejected_unknown_model = false;
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
