#ifndef UNITREE_DOG_H
#define UNITREE_DOG_H
#include <iostream>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <stdexcept>
#include <memory>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include "robot_hardware_interface.h"
using namespace std;
struct TestOption
{
    std::string name;
    int id;
};
class UnitreeDog : public RobotHardwareInterface
{
public:
    UnitreeDog() {}
    UnitreeDog(YAML::Node config);
    ~UnitreeDog() = default;

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;
 private:
    std::unique_ptr<unitree::robot::b2::SportClient> sport_client_;
    std::string network_interface_card_name_;
    float timeout_;
    vector<TestOption> option_list_;
    int ConvertToInt(const std::string &str);
    int32_t standUp();
    int32_t lieDown();
    int32_t stopMove();
    bool is_initialized_;
};

#endif