#ifndef ZSIBOT_ZSL_ONE_H
#define ZSIBOT_ZSL_ONE_H

#include "robot_hardware_interface.h"
#include "highlevel.h"
#include <iostream>

#include <thread>
enum CtrlMode {
    CTRLMODE_MOTOR_DAMPING = 0,//设备趴下，电机进入阻尼状态，卧倒，肚子挨着地
    CTRLMODE_STAND_UP = 1,//站立状态/打招呼状态
    CTRLMODE_MOTOR_FREE = 10,//设备趴下，电机自由状态
    CTRLMODE_MOVING = 18,//设备移动状态
    CTRLMODE_ACTION = 21,//设备动作状态(站立/打招呼/跳跃/前空翻/后空翻)
    CTRLMODE_LIE_DOWN = 51,//狗趴下状态
};

class ZsibotZslOne : public RobotHardwareInterface
{
public:
    ZsibotZslOne() {}
    ZsibotZslOne(YAML::Node config);
    ~ZsibotZslOne() = default;

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;
    mc_sdk::zsl_1::HighLevel highlevel_;
private:
    int32_t parseActionResult(uint32_t ret);
    // ========== TODO config补充Zsibot缺失的成员变量 ==========
    std::string dog_ip_;          // 机器人IP地址
    std::string client_ip_;       // 客户端IP地址
    int client_port_;             // 客户端端口号
    bool is_initialized_;         // 标记机器人是否初始化成功
    int32_t standUp();
    int32_t lieDown();
    
};

#endif