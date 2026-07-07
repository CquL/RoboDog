# robot_hardware 代码架构说明

本文档用于说明 `robot_hardware` 包的整体代码架构：上层控制代码、硬件抽象接口、机器人适配类之间如何协作，以及后续如何接入新的机器人型号。

## 1. 这个包的定位

`robot_hardware` 不是完整的导航算法库，也不是感知算法库。它的定位是：

```text
机器人硬件抽象层 Hardware Abstraction Layer, HAL
```

它的目标是把不同机器人的底层 SDK、ROS 话题、UDP 协议等差异封装起来，对上层算法暴露一套统一接口。

也就是说，上层算法只关心：

```text
初始化机器人
发送速度命令
发送动作命令
```

而不用直接关心底层到底是宇树 SDK、智身 SDK，还是云深处 X30 的 ROS/UDP 控制接口。

## 2. 总体分层

当前代码可以分成四层：

```text
上层控制层
  robot_test.cpp
  未来我们的导航、规划、协同控制算法

配置与工厂层
  config.yaml
  robot_factory.h

抽象接口层
  robot_hardware_interface.h
  robot_hardware_constant.h
  robot_hardware_error_code.h

具体机器人适配层
  unitree/unitree_dog.h + unitree_dog.cpp
  zsibot/zsibot_zsl_one.h + zsibot_zsl_one.cpp
  deep_robotics/deep_robotics_x30.h + deep_robotics_x30.cpp
```

调用关系如下：

```text
config.yaml
  ↓
上层程序读取配置
  ↓
RobotFactory::RobotAllocate(config)
  ↓
创建 UnitreeDog / ZsibotZslOne / DeepRoboticsX30
  ↓
返回 RobotHardwareInterface 指针
  ↓
上层调用统一接口
  ↓
具体机器人适配类调用各自底层 SDK / ROS / UDP
  ↓
真实机器人执行
```

## 3. 核心抽象接口

核心文件：

```text
include/robot_hardware_interface.h
```

里面定义了统一速度命令：

```cpp
struct RobotVelocityCommand {
    double vx, vy, omega;
};
```

含义：

```text
vx      机器人前后方向速度
vy      机器人左右方向速度
omega   机器人 yaw 角速度
```

同时定义了所有机器人都必须实现的抽象接口：

```cpp
class RobotHardwareInterface
{
public:
    RobotHardwareInterface() {}
    RobotHardwareInterface(YAML::Node config) : config_(config) {}
    virtual ~RobotHardwareInterface() = default;

    virtual int32_t initRobotHardware() = 0;
    virtual int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) = 0;
    virtual int32_t writeActionCommand(std::string action) = 0;

protected:
    YAML::Node config_;
};
```

三个接口的职责：

| 接口 | 作用 |
|---|---|
| `initRobotHardware()` | 初始化机器人硬件连接，例如 SDK、网络、串口、ROS 节点等 |
| `writeRobotVelocityCommand(cmd)` | 发送统一速度命令，底层适配类负责转换成对应机器人可执行的命令 |
| `writeActionCommand(action)` | 发送动作命令，例如站立、趴下、停止移动 |

这里的 `= 0` 表示纯虚函数。也就是说，`RobotHardwareInterface` 只是规定接口形式，不能直接实例化。每个具体机器人都必须继承它，并实现这些函数。

## 4. 配置与工厂

配置文件：

```text
config.yaml
```

示例：

```yaml
robot_model: "unitree_dog"
network_interface_card_name: "eth0"
timeout: 25.0
```

`robot_model` 决定创建哪一种机器人适配对象。

工厂文件：

```text
include/robot_factory.h
```

核心逻辑：

```cpp
if(robot_type == "unitree_dog"){
    return std::make_shared<UnitreeDog>(node);
}else if(robot_type == "zsibot_zsl_one"){
    return std::make_shared<ZsibotZslOne>(node);
}else if(robot_type == "deep_robotics_x30"){
    return std::make_shared<DeepRoboticsX30>(node);
}
```

这就是工厂模式。上层程序不直接 `new UnitreeDog` 或 `new DeepRoboticsX30`，而是把配置交给 `RobotFactory`，由工厂根据配置创建具体对象。

## 5. 上层如何调用

测试程序：

```text
robot_test.cpp
```

核心流程：

```cpp
YAML::Node config = YAML::LoadFile("../config.yaml");
std::shared_ptr<RobotHardwareInterface> robot =
    RobotFactory::RobotAllocate(config);

int init_ret = robot->initRobotHardware();

RobotVelocityCommand cmd;
cmd.vx = 0.3;
cmd.vy = 0.0;
cmd.omega = 0.0;

robot->writeRobotVelocityCommand(cmd);
```

这里的关键点是：

```cpp
std::shared_ptr<RobotHardwareInterface> robot
```

上层拿到的是抽象接口指针，不是某个具体机器人类型。因此，上层算法不需要知道底层到底是哪一款机器人。

只要底层适配类实现正确，上层始终可以统一调用：

```text
robot->initRobotHardware()
robot->writeRobotVelocityCommand(cmd)
robot->writeActionCommand(action)
```

## 6. 当前已有机器人适配

### 6.1 UnitreeDog

文件：

```text
include/unitree/unitree_dog.h
src/unitree/unitree_dog.cpp
```

当前实现方式：

```text
RobotHardwareInterface
  ↓
UnitreeDog
  ↓
unitree::robot::b2::SportClient
  ↓
SportClient::Move(vx, vy, omega)
```

速度控制最终调用：

```cpp
sport_client_->Move(cmd.vx, cmd.vy, cmd.omega);
```

动作控制包括：

```text
stand_up   -> BalanceStand()
lie_down   -> Damp()
stop_move  -> StopMove()
```

### 6.2 ZsibotZslOne

文件：

```text
include/zsibot/zsibot_zsl_one.h
src/zsibot/zsibot_zsl_one.cpp
include/zsibot/highlevel.h
lib/zsibot/
```

当前实现方式：

```text
RobotHardwareInterface
  ↓
ZsibotZslOne
  ↓
mc_sdk::zsl_1::HighLevel
  ↓
highlevel_.move(vx, vy, omega)
```

速度控制最终调用：

```cpp
highlevel_.move(cmd.vx, cmd.vy, cmd.omega);
```

动作控制包括：

```text
stand_up -> highlevel_.standUp()
lie_down -> highlevel_.passive()
```

这个适配类依赖 `lib/zsibot` 下面的二进制 SDK 库。

### 6.3 DeepRoboticsX30

文件：

```text
include/deep_robotics/deep_robotics_x30.h
src/deep_robotics/deep_robotics_x30.cpp
```

当前状态：

```text
已有类定义
已有接口函数
但具体实现还是空壳
```

当前代码：

```cpp
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
```

这说明 X30 适配类目前还不能真正控制机器狗。后续需要在这里补充 X30 的真实控制逻辑。

## 7. 新增机器人时需要做什么

新增一个机器人型号时，推荐步骤：

```text
1. 在 include/ 下新建机器人头文件
2. 在 src/ 下新建机器人实现文件
3. 新类继承 RobotHardwareInterface
4. 实现 initRobotHardware()
5. 实现 writeRobotVelocityCommand()
6. 实现 writeActionCommand()
7. 在 robot_factory.h 中注册 robot_model
8. 在 CMakeLists.txt 中加入必要依赖和库
9. 在 config.yaml 中配置 robot_model 和底层连接参数
```

示例结构：

```text
include/new_robot/new_robot.h
src/new_robot/new_robot.cpp
```

示例类：

```cpp
class NewRobot : public RobotHardwareInterface
{
public:
    NewRobot(YAML::Node config);

    int32_t initRobotHardware() override;
    int32_t writeRobotVelocityCommand(RobotVelocityCommand &cmd) override;
    int32_t writeActionCommand(std::string action) override;
};
```

然后在 `robot_factory.h` 里加入：

```cpp
else if(robot_type == "new_robot"){
    return std::make_shared<NewRobot>(node);
}
```

这样上层算法不需要重写，只需要在配置文件里切换：

```yaml
robot_model: "new_robot"
```

## 8. X30 后续适配建议

对于云深处 X30 Pro，不建议直接绕过原厂底层控制。推荐路线是：

```text
上层算法
  ↓
RobotHardwareInterface
  ↓
DeepRoboticsX30
  ↓
发布 ROS2 /cmd_vel 或桥接到原厂 ROS1 /cmd_vel
  ↓
原厂地形图、安全层、速度修正
  ↓
运动主机执行
```

也就是说，X30 的 `writeRobotVelocityCommand()` 后续可以实现为：

```text
接收 RobotVelocityCommand
转换为 geometry_msgs/Twist
发布到 /cmd_vel 或指定控制话题
```

X30 的 `writeActionCommand()` 后续可以实现为：

```text
stand_up   -> 调用原厂站立接口或 UDP 手柄动作
lie_down   -> 调用原厂趴下接口或 UDP 手柄动作
stop_move  -> 发布零速度或调用停止接口
```

需要注意：

```text
1. 当前 DeepRoboticsX30 还没有真实控制逻辑。
2. 不要直接关闭原厂地形图安全层。
3. 优先通过原厂已经封装好的 ROS/导航/速度接口接入。
4. 上层算法只依赖 RobotHardwareInterface，不直接写死 X30 细节。
```

## 9. 文件作用速查

| 文件 | 作用 |
|---|---|
| `config.yaml` | 配置当前使用哪一种机器人，以及连接参数 |
| `robot_test.cpp` | 示例上层程序，演示如何通过统一接口控制机器人 |
| `include/robot_hardware_interface.h` | 统一硬件抽象接口 |
| `include/robot_factory.h` | 根据配置创建具体机器人对象 |
| `include/robot_hardware_constant.h` | 动作命令字符串常量 |
| `include/robot_hardware_error_code.h` | 返回码和错误码 |
| `src/robot_hardware_constant.cpp` | 动作命令常量定义 |
| `include/unitree/unitree_dog.h` | 宇树机器人适配类声明 |
| `src/unitree/unitree_dog.cpp` | 宇树机器人适配实现 |
| `include/zsibot/zsibot_zsl_one.h` | Zsibot 机器人适配类声明 |
| `src/zsibot/zsibot_zsl_one.cpp` | Zsibot 机器人适配实现 |
| `include/deep_robotics/deep_robotics_x30.h` | X30 适配类声明 |
| `src/deep_robotics/deep_robotics_x30.cpp` | X30 适配实现，目前为空壳 |
| `CMakeLists.txt` | 构建动态库、测试程序、链接 SDK 依赖 |

## 10. 一句话总结

`robot_hardware` 的核心思想是：

```text
上层算法面向 RobotHardwareInterface 编程；
每个机器人在底层实现自己的适配类；
新增机器人时主要新增适配类和配置，不重写上层控制算法。
```

