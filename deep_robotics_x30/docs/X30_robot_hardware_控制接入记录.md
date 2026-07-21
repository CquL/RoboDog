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

## 11. X30 UDP 测试准备与上机步骤

本地已经准备了 X30 UDP 控制测试相关文件：

```text
robot_hardware/robot_hardware/include/deep_robotics/deep_robotics_x30.h
robot_hardware/robot_hardware/src/deep_robotics/deep_robotics_x30.cpp
robot_hardware/robot_hardware/robot_test_x30_udp.cpp
robot_hardware/robot_hardware/config.yaml
robot_hardware/robot_hardware/CMakeLists.txt
```

当前测试程序 `robot_test_x30_udp.cpp` 有两个模式：

```text
zero  默认模式，只初始化、发零速度，不让机器狗动。
move  显式运动模式，发送 vx=0.10 m/s，持续 1 秒，然后停止。
```

上机测试前必须确认：

```text
1. 机器狗处于安全空旷区域。
2. 手柄在旁边，随时可以急停/接管。
3. 先运行 zero 模式。
4. zero 模式确认无异常后，再考虑 move 模式。
```

### 11.1 传输代码到机器狗

本地已经可以打包：

```powershell
cd D:\Desktop\RoboDog
tar -czf robot_hardware_x30_udp_transfer.tar.gz robot_hardware
scp .\robot_hardware_x30_udp_transfer.tar.gz ysc@192.168.1.106:/home/ysc/
```

机器狗上解压：

```bash
cd /home/ysc
rm -rf robot_hardware
tar -xzf robot_hardware_x30_udp_transfer.tar.gz
cd /home/ysc/robot_hardware/robot_hardware
```

### 11.2 编译 X30-only 版本

为避免机器狗上缺少 Unitree SDK 或 Zsibot SDK，当前推荐只编译 X30 UDP 测试：

```bash
cd /home/ysc/robot_hardware/robot_hardware
rm -rf build_x30
mkdir build_x30
cd build_x30
cmake -DBUILD_X30_ONLY=ON ..
make -j$(nproc)
```

编译成功后应生成：

```text
build_x30/robot_test_x30_udp
```

### 11.3 零速度安全测试

先只运行 zero 模式：

```bash
cd /home/ysc/robot_hardware/robot_hardware/build_x30
./robot_test_x30_udp ../config.yaml zero
```

期望：

```text
程序初始化 UDP 成功。
持续发送心跳和零速度。
机器狗不运动。
程序正常退出。
```

### 11.4 极低速运动测试

只有 zero 模式确认正常后，才运行 move 模式：

```bash
cd /home/ysc/robot_hardware/robot_hardware/build_x30
./robot_test_x30_udp ../config.yaml move
```

期望：

```text
机器狗以很小速度前进约 1 秒。
随后程序连续发布零速度并停止。
```

如果方向反了，需要检查：

```text
invert_vy_axis
invert_omega_axis
X30 前进轴指令映射
```

### 11.5 测试后记录

每次测试后记录：

```text
1. 编译是否成功。
2. zero 模式是否无运动。
3. move 模式是否按预期小幅前进。
4. 是否能停止。
5. 是否需要调整轴方向或限幅。
```

## 12. UDP 初测结果与下一步判断

本次在机器狗上已经完成 X30 UDP 测试程序编译和运行。

执行结果：

```text
./robot_test_x30_udp ../config.yaml zero
  程序初始化成功。
  发送零速度后退出。
  机器狗没有运动，符合预期。

./robot_test_x30_udp ../config.yaml move
  程序初始化成功。
  发送小速度命令。
  程序正常停止退出。
  机器狗没有明显运动。
```

已确认原厂 ROS1 控制相关话题存在：

```text
/cmd_vel
/cmd_vel_corrected
/handle_state
/robot_velocity
/control_mode
/mode_query
/mode_setting
/robot_gait_state
```

关键话题关系：

```text
/cmd_vel:
  Publishers:
    /udp_receiver
    /robot_server
    /move_base
  Subscribers:
    /udp_sender
    /robot_server

/cmd_vel_corrected:
  Publisher:
    /DwaLocalPlanner
  Subscribers:
    /udp_sender
    /robot_server

/handle_state:
  Publisher:
    /udp_receiver

/robot_velocity:
  Publisher:
    /udp_receiver
```

当前判断：

```text
1. UDP 测试程序没有报错，只能说明本机 sendto 成功。
2. 机器狗没有运动，说明运动主机没有执行这组 UDP 轴值。
3. 原因可能是机器人没有处于踏步/开始运动状态，或 UDP 指令包格式/状态机还需要继续核对。
4. 由于 /cmd_vel 已经有 /udp_sender 订阅，ROS1 /cmd_vel 可能是更稳的 X30 控制入口。
```

下一步建议：

```text
先不要继续盲目运行 UDP move。
先读取 /control_mode、/mode_query、/robot_gait_state、/robot_basic_state。
再用 ROS /cmd_vel 做零速度验证。
如果 ROS /cmd_vel 链路可用，优先把 DeepRoboticsX30 改成 ROS /cmd_vel 后端。
```

## 13. UDP 轴值链路和导航链路的区别

根据《绝影X30接口规格书（对外）ros+udp V1.0.4》继续核对后，当前判断如下：

```text
1. UDP 轴指令更像“模拟手柄/遥控终端”。
2. 运动主机手柄/轴指令前缀应使用 0x21，不是 0x31。
3. 手动模式下，机器人直接响应手柄摇杆轴指令。
4. 非手动模式下，机器人根据地形图速度输入源进入辅助/导航链路。
5. 所以 UDP 轴指令测试和后续自主导航算法不应该混成一条链路。
```

已修正 X30 UDP 代码：

```text
心跳：0x21040001
确认连接：0x21020001
非手动模式：0x21010C03
开始运动/停止运动：0x21010201
前后速度轴：0x21010130
左右速度轴：0x21010131
Yaw 速度轴：0x21010135
感知主机速度源：0x3101EE03
```

当前 UDP 测试策略：

```text
1. 不自动切非手动/导航模式。
2. 保持手动控制链路。
3. move 测试时先发送一次 0x21010201，让机器人从力控站立进入踏步。
4. 发送极小前进轴值，持续 800 ms。
5. 连续发送零速度。
6. 再发送一次 0x21010201，让机器人从踏步回到力控站立。
```

当前 `config.yaml` 中用于 UDP 轴值测试的设置：

```yaml
motion_command_source: "navigation"
configure_non_manual_mode: false
configure_navigation_velocity_source: false
```

2026-07-07 继续测试记录：

```text
测试环境：
  /control_mode = 0
  /robot_basic_state = 3

测试命令：
  ./robot_test_x30_udp ../config.yaml zero
  ./robot_test_x30_udp ../config.yaml move

测试结果：
  zero 模式无误动。
  move 模式执行了 start/stop motion toggle、小速度、零速度、再次 toggle。
  机器狗仍未明显运动。
```

新的判断：

```text
仅使用 0x21 手柄前缀从 106 主机发包，运动主机可能不接收或不按预期执行。
根据接口书说明，导航模块/自主移动算法模块下发指令码时前两位应使用 0x31。
因此代码改为支持 motion_command_source 配置：
  navigation -> 0x31 前缀
  remote     -> 0x21 前缀
当前默认使用 navigation。
```

2026-07-07 继续测试结果：

```text
配置：
  motion_command_source: "navigation"

测试命令：
  ./robot_test_x30_udp ../config.yaml zero
  ./robot_test_x30_udp ../config.yaml move

结果：
  zero 模式无误动。
  move 模式机器狗已经发生小幅运动。
  程序输出显示 motion_command_source=navigation。
```

测试时状态观察：

```text
/control_mode:
  data: 0

/robot_basic_state:
  data: 16

/robot_velocity:
  linear.x/y/z 仍接近 0
  angular 有极小漂移
```

接口规格书状态表：

```text
/control_mode = 0：手动模式
/control_mode = 1：非手动模式

/robot_basic_state = 3：力控站立状态
/robot_basic_state = 4：踏步状态
/robot_basic_state = 16：RL 状态
```

当前判断：

```text
1. 从 106 主机发送 UDP 控制命令时，navigation/0x31 前缀是有效的。
2. 机器狗运动时未显示为 4 踏步状态，而是显示为 16 RL 状态。
3. /robot_velocity 没有明显 linear 变化，可能是测试速度和时长太小，或者该话题不直接反映这条 UDP 控制链路的瞬时命令。
4. 当前已经证明 DeepRoboticsX30 的 UDP 后端可以让 X30 产生运动。
```

2026-07-07 进一步人工模式验证：

```text
现象：
  遥控器/APP 调到手动模式时，robot_test_x30_udp move 可以让机器狗小幅运动。
  遥控器/APP 调到导航模式时，同样的 UDP move 不运动。

结论：
  当前 DeepRoboticsX30 UDP 后端本质上是“轴值/手柄类控制链路”。
  它适合在手动模式下验证底层运动控制，不适合作为导航模式下的主控制入口。
  导航模式下原厂系统更可能等待导航速度、路径点或地形图修正后的速度，而不是直接执行这组 UDP 轴值。
```

后续自主导航算法建议走另一条链路：

```text
自研导航算法
  -> 发布 ROS /cmd_vel
  -> 原厂地形图/安全层
  -> /cmd_vel_corrected
  -> 原厂 udp_sender
  -> 运动主机执行
```

也就是说：

```text
UDP 轴值：用于模拟手柄、验证底层运动接入。
ROS /cmd_vel：用于后续自己的导航算法和上层控制。
```

### 13.1 motion_command_source 和遥控器导航模式不是一回事

`config.yaml` 中的：

```yaml
motion_command_source: "navigation"
```

只表示 UDP 指令码前缀使用 `0x31...`，含义是“从导航/自主模块这类发送源发出的 UDP 指令”。

这不等于遥控器/APP 右侧按钮里的“导航模式”。

当前区分：

```text
motion_command_source: "navigation"
  代码层含义：UDP 指令码前缀为 0x31。
  作用对象：发给运动主机的 UDP 命令格式。

遥控器/APP 导航模式
  系统层含义：进入原厂非手动/导航链路。
  作用对象：原厂导航、地形图、安全层、路径执行。
```

实测现象：

```text
遥控器/APP 手动模式：
  当前 UDP 轴值后端可以让机器狗小幅运动。

遥控器/APP 导航模式：
  当前 UDP 轴值后端不运动。
```

因此后续实现要分清两类控制：

```text
1. UDP 轴值后端：
   适合手动模式下验证 robot_hardware 能否控制 X30。
   不建议作为自研导航算法主链路。

2. ROS /cmd_vel 后端：
   适合自研导航算法输出速度。
   应该配合原厂非手动/导航模式、地形图安全层、/cmd_vel_corrected 使用。
```

### 13.2 如果不用 ROS，是否可以直接复刻原厂 UDP 控制

当前判断：

```text
可以，但需要复刻的是“原厂 udp_sender/运动链路的完整 UDP 行为”，不是只发一个 vx/vy/omega。
```

原厂导航链路很可能是：

```text
move_base / DWA / 导航模块
  -> 计算导航速度
  -> 地形图/安全层修正速度
  -> 原厂 udp_sender
  -> 打包 UDP 指令
  -> 192.168.1.103:43893 运动主机
```

所以从底层看，最后确实仍然会落到 UDP 包。但当前我们写的 UDP 后端只是：

```text
简单心跳
简单连接确认
开始/停止运动 toggle
三轴摇杆轴值
```

它已经能在手动模式下让 X30 动，说明底层 UDP 通路打通了。

但它还不能说明我们已经完整复刻了原厂导航模式下的 UDP 发送器。

如果后续坚持“不使用 ROS”，推荐下一步做：

```text
1. 抓包原厂 udp_sender 在手动/导航/路径执行时发给 192.168.1.103:43893 的 UDP 包。
2. 对比我们当前 DeepRoboticsX30 发出的 UDP 包。
3. 明确原厂导航模式下是否仍然发送三轴轴值，还是发送了其他状态/模式/速度结构。
4. 再把 DeepRoboticsX30 扩展成 x30_udp_nav 后端。
```

抓包验证比继续猜指令码更可靠。

## 14. 2026-07-10 `/vel_source` 实机更正

历史记录中曾假设可以使用：

```bash
rosparam set /vel_source 2
```

2026-07-10 在 `192.168.1.106` 实机采集后确认：

```text
/vel_source       不是当前存在的 ROS 参数
/vel_source       话题当前也未广播
/vel_source_state 话题当前也未广播
```

规格书第 40 页将 `/vel_source` 定义为 `std_msgs/Int32` 话题，不是 ROS 参数。因此历史记录中的 `rosparam set` 命令不再使用。

当前原厂运行拓扑：

```text
/cmd_vel, /cmd_vel_corrected
  -> /home/ysc/jy_cog/transfer/lib/message_transformer_cpp/udp_sender
  -> 192.168.1.103:43893

192.168.1.103
  -> 192.168.1.106:43897
  -> udp_receiver
  -> /robot_velocity, /leg_odom, /robot_basic_state, /robot_gait_state
```

完整采集结果和下一步抓包流程见：

```text
docs/X30_UDP速度一致性根因与原厂接口调研_20260710.md
```

## 15. 2026-07-10 原厂物理速度 UDP 已确认

已将机器狗原厂文件复制到本地：

```text
D:\Desktop\RoboDog\message_transformer_cpp
D:\Desktop\RoboDog\launch
```

通过 PCAP 和带调试符号的原厂二进制交叉确认：

```text
/cmd_vel_corrected
  vx    -> 0x123, int32(vx * 1000)
  vy    -> 0x124, int32(vy * 1000)
  omega -> 0x122, int32(omega * 1000)
  -> 192.168.1.103:43893

/cmd_vel
  -> code=0x150, size=24, type=1
  -> double vx, double vy, double omega
  -> 192.168.1.105:43897
  -> 105 原厂地形图/安全修正链
```

同时更正第 14 节：`/vel_source` 在 106 ROS 图上不可见，但 105 的 `app_port` 二进制明确发布 `/vel_source`、订阅 `/vel_source_state`；UDP `0x3101EE03` 是 105:43899 的速度源设置入口。它仍然不是 ROS 参数。

`DeepRoboticsX30` 现支持：

```text
udp_navigation  默认，复刻 /cmd_vel 跨主机 UDP，保留 105 安全层
udp_physical    复刻 /cmd_vel_corrected，直接发运动主机
udp_axis        保留旧摇杆轴值测试
```

当前只完成离线协议验证。下一次先在机器狗运行 `zero` 并抓取 `105:43897`，确认 36 字节 `0x150` 零速度包；验证前不运行非零速度测试。
