# X30 robot_hardware 控制接入记录

更新时间：2026-07-07  
项目路径：`D:\Desktop\RoboDog`  
相关代码：`D:\Desktop\RoboDog\robot_hardware`

## 1. 本次准备做什么

我们已经完成了 ROS2 Docker 对 X30 四雷达、点云融合、本体 IMU 的数据接入。下一步要把 X30 接入 `robot_hardware`，让上层算法可以通过统一硬件接口控制 X30。

目标调用方式：

```cpp
std::shared_ptr<RobotHardwareInterface> robot =
    RobotFactory::RobotAllocate(config);

robot->initRobotHardware();

RobotVelocityCommand cmd;
cmd.vx = 0.2;
cmd.vy = 0.0;
cmd.omega = 0.0;

robot->writeRobotVelocityCommand(cmd);
```

上层算法只调用 `RobotHardwareInterface`，不直接关心底层是宇树、智身，还是云深处 X30。

## 2. 当前代码状态

当前 X30 适配文件：

```text
robot_hardware/robot_hardware/include/deep_robotics/deep_robotics_x30.h
robot_hardware/robot_hardware/src/deep_robotics/deep_robotics_x30.cpp
```

当前 `DeepRoboticsX30` 已经继承了统一接口：

```cpp
class DeepRoboticsX30 : public RobotHardwareInterface
```

但是实现还是空壳：

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

结论：

```text
当前 deep_robotics_x30.cpp 只是占位适配类。
它不会真的向 X30 发布速度命令，也不会调用原厂动作接口。
```

## 3. 专业接入思路

X30 Pro 已经有原厂 ROS1 系统，包含运动控制、地形图安全层、导航、避障等能力。因此不建议直接绕过原厂底层运动控制。

推荐接入路线：

```text
上层算法
  ↓
RobotHardwareInterface
  ↓
DeepRoboticsX30
  ↓
发布速度命令 /cmd_vel
  ↓
原厂地形图/安全层修正
  ↓
/cmd_vel_corrected
  ↓
运动主机执行
```

也就是说，X30 的 `writeRobotVelocityCommand()` 不应该直接控制关节，而应该把统一速度命令转换成 X30 原厂系统能接收的速度话题。

初步建议：

```text
1. 优先使用 ROS 话题路线。
2. 速度命令发布到 /cmd_vel。
3. 设置 /vel_source = 2，让速度走导航/地形图安全层。
4. 不直接绕过 /cmd_vel_corrected。
5. 不直接接运动主机底层关节控制。
```

## 4. 上机后需要确认什么

连接机器狗 `192.168.1.106` 后，需要先确认原厂控制入口。

### 4.1 查看 ROS1 控制话题

```bash
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

rostopic list | grep -E "cmd_vel|cmd_vel_corrected|robot_velocity|move_base|vel_source|location_status"
```

重点关注：

```text
/cmd_vel
/cmd_vel_corrected
/robot_velocity
/move_base_simple/goal
/move_base/goal
/location_status
```

### 4.2 查看速度话题类型

```bash
rostopic info /cmd_vel
rostopic info /cmd_vel_corrected
rostopic info /robot_velocity
```

期望：

```text
/cmd_vel            geometry_msgs/Twist
/cmd_vel_corrected  geometry_msgs/Twist
/robot_velocity     geometry_msgs/Twist
```

### 4.3 查看速度源参数

```bash
rosparam get /vel_source
rosparam list | grep vel
```

如果需要让我们发布的 `/cmd_vel` 进入导航/地形图安全层，后续可能需要：

```bash
rosparam set /vel_source 2
```

实际是否设置，要以上机验证为准。

### 4.4 查看原厂启动脚本

```bash
find /home/ysc/jy_cog -iname "*vel*" -o -iname "*cmd*" -o -iname "*joy*" -o -iname "*motion*"
grep -R "cmd_vel" -n /home/ysc/jy_cog/system /home/ysc/jy_cog/drivers 2>/dev/null | head -50
grep -R "vel_source" -n /home/ysc/jy_cog/system /home/ysc/jy_cog/drivers 2>/dev/null | head -50
```

目的：

```text
确认 /cmd_vel 由谁订阅。
确认 /cmd_vel_corrected 由谁发布。
确认速度最终如何进入运动主机。
```

## 5. deep_robotics_x30.cpp 后续要实现什么

### 5.1 initRobotHardware()

职责：

```text
初始化 X30 控制通信。
```

可能实现方式：

```text
方案 A：ROS1 roscpp publisher
  在原厂 ROS1 环境中编译 robot_hardware。
  DeepRoboticsX30 创建 ros::Publisher，发布 geometry_msgs/Twist 到 /cmd_vel。

方案 B：ROS2 rclcpp publisher
  在我们的 ROS2 Docker 中编译 robot_hardware。
  DeepRoboticsX30 创建 rclcpp::Publisher，发布 geometry_msgs/msg/Twist。
  再通过 bridge 或原厂支持的接口进入 ROS1 控制系统。

方案 C：UDP 手柄协议
  直接向运动主机 192.168.1.103:43893 发送手柄轴值。
  这个更底层，风险更高，暂不作为第一选择。
```

当前推荐：

```text
优先方案 A 或 B。
不要一开始走 UDP 底层手柄协议。
```

### 5.2 writeRobotVelocityCommand()

职责：

```text
把 RobotVelocityCommand 转换为 X30 可执行速度命令。
```

目标逻辑：

```cpp
geometry_msgs::Twist twist;
twist.linear.x = cmd.vx;
twist.linear.y = cmd.vy;
twist.angular.z = cmd.omega;
publisher.publish(twist);
```

注意：

```text
1. 必须限制最大速度，避免误操作。
2. 默认测试速度要非常小，例如 vx = 0.05 ~ 0.10 m/s。
3. 每次测试结束必须连续发布零速度。
```

### 5.3 writeActionCommand()

职责：

```text
把统一动作命令转换为 X30 原厂动作控制。
```

当前动作常量：

```text
stand_up
lie_down
stop_move
```

短期建议：

```text
先只实现 stop_move：发布零速度。
stand_up / lie_down 暂时不接，继续用原厂手柄或原厂脚本。
```

原因：

```text
站立、趴下涉及机器人状态机和安全状态。
在没有确认原厂动作接口前，不建议盲目写。
```

## 6. 建议新增测试程序

后续可以新增一个 X30 专用测试文件：

```text
robot_hardware/robot_hardware/x30_robot_test.cpp
```

测试目标：

```text
1. 读取 config.yaml。
2. 创建 DeepRoboticsX30。
3. 初始化硬件接口。
4. 先只测试 stop_move。
5. 再测试极低速短时间移动。
6. 自动发布零速度停止。
```

测试流程建议：

```text
第一阶段：不让机器狗动
  只验证程序能启动、能发布 /cmd_vel、话题类型正确。

第二阶段：架空或安全环境
  发布 0 速度，确认不会误动。

第三阶段：空旷地面、人工手柄待命
  发布 vx=0.05 m/s，持续 1 秒。
  然后连续发布 0 速度。
```

## 7. 安全约束

X30 控制接入必须遵守：

```text
1. 测试时机器狗周围必须空旷。
2. 手柄必须在旁边，随时可以接管。
3. 初次测试不发布大速度。
4. 初次测试不实现站立/趴下动作。
5. 不绕过原厂地形图安全层。
6. 每个运动测试都必须自动发布零速度。
7. 每次测试前后记录当前 ROS 话题和参数状态。
```

## 8. 本次实际完成

截至本记录创建时：

```text
已完成：
1. 阅读 robot_hardware 代码结构。
2. 确认 DeepRoboticsX30 已继承 RobotHardwareInterface。
3. 确认 deep_robotics_x30.cpp 当前为空实现。
4. 梳理 X30 接入 robot_hardware 的推荐路线。
5. 明确上机后需要确认的 ROS 话题、参数、脚本。
6. 明确后续需要新增 X30 控制测试程序。

未完成：
1. 尚未连接机器狗进行 ROS 话题确认。
2. 尚未修改 deep_robotics_x30.cpp。
3. 尚未新增 x30_robot_test.cpp。
4. 尚未进行真实速度控制测试。
```

## 9. 下一次连接机器狗后的执行顺序

```bash
# 1. 登录机器狗
ssh ysc@192.168.1.106

# 2. 查看 ROS1 控制话题
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true
rostopic list | grep -E "cmd_vel|cmd_vel_corrected|robot_velocity|move_base|location_status"

# 3. 查看话题类型和订阅关系
rostopic info /cmd_vel
rostopic info /cmd_vel_corrected
rostopic info /robot_velocity

# 4. 查看速度源参数
rosparam get /vel_source
rosparam list | grep vel

# 5. 搜索原厂脚本中对速度话题的使用
grep -R "cmd_vel" -n /home/ysc/jy_cog/system /home/ysc/jy_cog/drivers 2>/dev/null | head -50
grep -R "vel_source" -n /home/ysc/jy_cog/system /home/ysc/jy_cog/drivers 2>/dev/null | head -50
```

确认结果后，再决定 `DeepRoboticsX30` 使用 ROS1、ROS2 bridge，还是其它接口实现。

## 10. X30 控制接口方式判断

根据本地资料 `绝影X30接口规格书（对外）ros+udp V1.0.4(1).pdf`，X30 对外控制主要有两类方式：

```text
1. ROS 话题方式
2. UDP 指令方式
```

接口规格书中明确写到：

```text
运动主机、感知主机、手柄之间通过 UDP 通信。
手柄或感知主机向运动主机发送消息的目标地址是 192.168.1.103:43893。
```

同时，感知主机 ROS 话题中包含：

```text
/cmd_vel            导航模块规划的运动速度，geometry_msgs/Twist
/cmd_vel_corrected  地形图发布的修正后的速度指令，geometry_msgs/Twist
/vel_source         设置速度输入源，std_msgs/Int32
/vel_source_state   查看当前速度输入源，std_msgs/Int32
```

规格书还说明，在辅助模式或导航模式下，运动程序执行的是 `/cmd_vel_corrected` 中发布的速度指令。也就是说，如果我们希望保留地形图碰撞检测和安全停避障，应该让速度先进入 `/cmd_vel`，再由地形图模块处理成 `/cmd_vel_corrected`。

因此当前工程判断如下：

| 方式 | 作用 | 是否推荐作为第一版 X30 robot_hardware 控制 |
|---|---|---|
| ROS1 `/cmd_vel` | 发布导航速度，走原厂地形图安全层 | 推荐 |
| ROS2 `/cmd_vel` + ros1_bridge | ROS2 算法输出，经桥接进入 ROS1 原厂控制链 | 推荐，但需要维护 bridge |
| UDP `192.168.1.103:43893` | 模拟手柄/感知主机向运动主机发控制指令 | 可作为底层备选，不建议第一版直接用 |
| 官方 `robotserver_sdk` | 连接 X30 控制系统、发送/取消/查询导航任务 | 更适合导航任务，不是第一版速度控制主接口 |
| 直接关节控制/运动主机底层控制 | 绕过原厂安全层 | 不推荐 |

当前推荐路线：

```text
第一版 DeepRoboticsX30:
  先实现 ROS 话题后端。
  writeRobotVelocityCommand() 将 RobotVelocityCommand 转成 geometry_msgs/Twist。
  发布到 /cmd_vel。
  /vel_source 设置为 2，让速度输入源为导航模块。
  让原厂地形图模块输出 /cmd_vel_corrected。

后续版本:
  如确实需要脱离 ROS1，可再实现 UDP 后端。
```

需要注意：

```text
robotserver_sdk 是官方公开 SDK，但它的 README 描述重点是控制和监控导航任务，
例如连接控制系统、获取状态、发送导航任务、取消导航任务、查询导航任务状态。
它不等同于宇树 SportClient 那种直接 Move(vx, vy, omega) 的运动 SDK。
```

