#ifndef ROBOT_HARDWARE_INTERFACE_H
#define ROBOT_HARDWARE_INTERFACE_H

// 上层导航代码只需包含此机器人边界头文件；具体实现将厂商传输封装在内部。

#include <math.h>
#include <stdint.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "robot_hardware_error_code.h"
#include "robot_hardware_constant.h"

// 厂商无关的机身坐标速度命令，单位为 SI：+x 向前，+y 向左，omega 正值逆时针。
struct RobotVelocityCommand {
    double vx, vy, omega;
};

// 上层控制器使用的通用 HAL 边界。具体机器人类将这些操作转换为厂商 API，
// 不让厂商协议进入导航代码。该接口只负责写入，不统一反馈；各平台通过不同
// middleware 发布状态，需要安全确认的调用方应单独订阅。
class RobotHardwareInterface
{
public:
    RobotHardwareInterface() {}
    RobotHardwareInterface(YAML::Node config) : config_(config) {}
    virtual ~RobotHardwareInterface() = default;

    // 初始化所选厂商传输及其安全状态。
    virtual int32_t initRobotHardware() = 0;

    // 发送一条机身坐标速度命令。
    virtual int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) = 0;

    // 发送一个具名机器人动作，例如步态切换。
    virtual int32_t writeActionCommand(std::string action) = 0;
protected:
    // 保留解析后的机器人配置供具体适配器使用，新增厂商参数无需扩展通用构造接口。
    YAML::Node config_;
 
};

#endif
