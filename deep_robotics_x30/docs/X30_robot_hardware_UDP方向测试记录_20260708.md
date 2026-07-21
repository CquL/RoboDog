# X30 robot_hardware UDP 方向测试记录

更新时间：2026-07-08  
项目路径：`D:\Desktop\RoboDog`  
相关代码：`D:\Desktop\RoboDog\robot_hardware\robot_hardware`

## 1. 本次准备做什么

当前 `DeepRoboticsX30` 已经实现 UDP 轴值后端。为了验证统一接口：

```cpp
robot->writeRobotVelocityCommand(cmd);
robot->writeActionCommand(action);
```

在 X30 上的方向映射是否正确，本次扩展：

```text
robot_hardware/robot_hardware/robot_test_x30_udp.cpp
```

让它支持多个安全方向测试模式。

默认仍然是 `zero`，不会运动。所有运动测试都必须显式指定模式。

## 2. 接口规格书依据

根据《绝影X30接口规格书（对外）ros+udp V1.0.4》：

```text
指令发送源前缀：
  0x21  手柄/遥控终端
  0x31  导航模块/自主移动算法模块

心跳：
  0x21040001

确认连接：
  0x21020001

开始运动/停止运动：
  0x21010201
  在力控站立状态和踏步状态之间轮流切换

X 方向线速度轴：
  0x21010130
  正值向前，负值向后

Y 方向线速度轴：
  0x21010131
  规格书轴值正值向右，负值向左

Yaw 角速度轴：
  0x21010135
  规格书轴值正值向右转，负值向左转
```

规格书还说明：

```text
手动模式下，机器人直接响应手柄/轴值速度指令。
非手动模式下，速度输入源进入辅助/导航链路。
辅助/导航模式下，运动程序执行 /cmd_vel_corrected。
```

因此当前 UDP 方向测试属于：

```text
UDP 轴值/手柄类控制链路
```

不等同于后续自主导航推荐使用的：

```text
ROS /cmd_vel -> 地形图安全层 -> /cmd_vel_corrected
```

## 3. 本次代码修改

修改文件：

```text
robot_hardware/robot_hardware/robot_test_x30_udp.cpp
```

新增测试：

```text
zero       只初始化和发零速度，不运动
list       打印可用模式，不初始化机器人
forward    小速度前进
backward   小速度后退
left       小速度向左平移
right      小速度向右平移
yaw_left   小角速度左转
yaw_right  小角速度右转
all_safe   按顺序执行所有方向测试
forward_1m 理论前进 1m
backward_1m 理论后退 1m
left_1m    理论左移 1m
right_1m   理论右移 1m
all_1m     按顺序执行所有 1m 线性距离测试
```

兼容旧命令：

```text
move       等价于 forward
turn_left  等价于 yaw_left
turn_right 等价于 yaw_right
```

新增测试文件：

```text
robot_hardware/robot_hardware/tests/test_x30_udp_direction_modes.py
```

本地已验证：

```text
python -m pytest .\robot_hardware\robot_hardware\tests\test_x30_udp_direction_modes.py
结果：2 passed
```

## 4. 当前安全参数

测试程序内置低速参数：

```text
vx    = 0.05 m/s
vy    = 0.03 m/s
omega = 0.15 rad/s
持续时间 = 800 ms
```

1m 线性距离测试参数：

```text
目标距离 = 1.0 m
vx       = 0.10 m/s
vy       = 0.10 m/s
持续时间 = 10 s
```

没有直接把 `kSafeTestDurationMs` 改成 10 秒，原因是：

```text
原来的 yaw_left / yaw_right 也共用 kSafeTestDurationMs。
如果直接改成 10 秒，原地转向会持续太久，不适合作为安全方向确认测试。
因此保留短方向测试，额外新增 1m 线性距离测试。
```

每个方向测试流程：

```text
1. 连续发送零速度
2. 发送 start/stop motion toggle
   期望：力控站立 -> 踏步
3. 持续 800 ms 发送对应方向速度
4. 连续发送零速度
5. 发送 stop_move
6. 再发送 start/stop motion toggle
   期望：踏步 -> 力控站立
7. 再连续发送零速度
```

注意：

```text
测试前应让机器狗处于力控站立状态。
如果机器狗已经在踏步状态，start/stop motion toggle 的效果可能相反。
```

## 5. 上机编译

本地打包：

```powershell
cd D:\Desktop\RoboDog
tar -czf robot_hardware_x30_udp_transfer.tar.gz robot_hardware
scp .\robot_hardware_x30_udp_transfer.tar.gz ysc@192.168.1.106:/home/ysc/
```

机器狗上解压：

```bash
cd /home/ysc
mv robot_hardware robot_hardware.bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
tar -xzf robot_hardware_x30_udp_transfer.tar.gz
cd /home/ysc/robot_hardware/robot_hardware
```

编译 X30-only：

```bash
rm -rf build_x30
mkdir build_x30
cd build_x30
cmake -DBUILD_X30_ONLY=ON ..
make -j$(nproc)
```

生成：

```text
/home/ysc/robot_hardware/robot_hardware/build_x30/robot_test_x30_udp
```

## 6. 推荐测试顺序

先看可用模式：

```bash
./robot_test_x30_udp ../config.yaml list
```

第一步，只跑零速度：

```bash
./robot_test_x30_udp ../config.yaml zero
```

确认无误动后，再逐个方向测试。不要一开始运行 `all_safe`。

```bash
./robot_test_x30_udp ../config.yaml forward
./robot_test_x30_udp ../config.yaml backward
./robot_test_x30_udp ../config.yaml left
./robot_test_x30_udp ../config.yaml right
./robot_test_x30_udp ../config.yaml yaw_left
./robot_test_x30_udp ../config.yaml yaw_right
```

如果要验证理论 1m 位移，先逐个运行：

```bash
./robot_test_x30_udp ../config.yaml forward_1m
./robot_test_x30_udp ../config.yaml backward_1m
./robot_test_x30_udp ../config.yaml left_1m
./robot_test_x30_udp ../config.yaml right_1m
```

每次测试后记录：

```text
1. 机器狗是否按模式名对应方向轻微运动。
2. 是否能自动停住。
3. 是否切回力控站立。
4. 是否有方向反了。
```

所有单方向都确认后，才考虑：

```bash
./robot_test_x30_udp ../config.yaml all_safe
```

所有 1m 单方向都确认后，才考虑：

```bash
./robot_test_x30_udp ../config.yaml all_1m
```

## 7. 方向反了如何判断

当前 `config.yaml`：

```yaml
invert_vy_axis: true
invert_omega_axis: false
motion_command_source: "navigation"
```

预期：

```text
forward    前进
backward   后退
left       向机器狗左侧平移
right      向机器狗右侧平移
yaw_left   左转
yaw_right  右转
```

如果左右平移反了：

```yaml
invert_vy_axis: false
```

或恢复为：

```yaml
invert_vy_axis: true
```

如果 yaw 左右反了：

```yaml
invert_omega_axis: true
```

或恢复为：

```yaml
invert_omega_axis: false
```

修改后重新编译不是必须的，因为配置运行时读取。

## 8. 当前不测试的内容

规格书中还有：

```text
行走步态
斜坡步态
越障步态
楼梯步态
L 行走步态
山地步态
静音步态
身体高度切换
手动/非手动模式切换
```

当前 `DeepRoboticsX30` 还没有正式封装这些动作，不纳入本次方向测试。

本次只验证：

```text
三轴速度 UDP 包
start/stop motion toggle
stop_move 零速度停止
```

## 9. 安全要求

```text
1. 测试区域必须空旷。
2. 遥控器/APP 必须在手边，随时能接管。
3. 先 zero，再单方向。
4. 每次只测一个方向。
5. 不在导航模式下盲目测试 UDP 轴值链路。
6. 不直接测试站立/趴下。
7. all_safe 只在六个单方向全部确认后使用。
```

## 10. 2026-07-10 步态确认与速度上限修正

机器狗宿主机 ROS1 状态话题确认结果：

```text
/control_mode:
  data: 0

/robot_basic_state:
  data: 16

/robot_gait_state:
  data: 32

/robot_velocity:
  linear.x/y/z = 0
```

根据 X30 接口规格书：

```text
/control_mode = 0：手动模式
/robot_basic_state = 16：RL 状态
/robot_gait_state = 32：L 行走步态
```

因此当前 UDP 方向测试应按 L 行走步态估算三轴速度上限：

```yaml
max_vx: 1.0
max_vy: 0.5
max_omega: 1.2
```

之前配置：

```yaml
max_vx: 0.5
max_vy: 0.3
max_omega: 0.5
```

会把速度命令换算成过大的轴值。例如 `cmd.vx = 0.10` 时，旧配置按 `0.10 / 0.5 = 20%` 轴值发送；若当前真实 L 行走最大前进速度为 `1.0 m/s`，实际前进速度约为 `0.20 m/s`，10 秒约为 2m，因此会明显超过 1m。

本地 `robot_hardware/robot_hardware/config.yaml` 已改为 L 行走步态速度上限。下一次传到机器狗后，先只测试：

```bash
./robot_test_x30_udp ../config.yaml zero
./robot_test_x30_udp ../config.yaml forward_1m
```

不要直接运行 `all_1m`。1m 测试仍是开环速度时间估算，不是闭环位移控制；最终精确位移需要结合 `/leg_odom` 或其他定位反馈。

后续实测发现，即使按 L 行走步态改为：

```yaml
max_vx: 1.0
max_vy: 0.5
max_omega: 1.2
```

`forward_1m` 仍有明显超距风险。因此不再推荐直接用 10 秒 1m 模式做第一轮标定。测试程序新增短脉冲标定模式：

```bash
./robot_test_x30_udp ../config.yaml forward_calib
./robot_test_x30_udp ../config.yaml backward_calib
./robot_test_x30_udp ../config.yaml left_calib
./robot_test_x30_udp ../config.yaml right_calib
```

短脉冲参数：

```text
vx = 0.05 m/s
vy = 0.05 m/s
duration = 1s
理论位移 = 0.05m
```

下一轮应先运行：

```bash
./robot_test_x30_udp ../config.yaml zero
./robot_test_x30_udp ../config.yaml forward_calib
```

测出实际位移后，再反推 X30 UDP 轴值链路在当前状态下的速度比例。不要继续盲目运行 `forward_1m`。

短脉冲实测：

```text
forward_calib:
  命令 vx = 0.05 m/s
  持续时间 = 1s
  /leg_odom 起点 x = -43.625850677490234
  /leg_odom 终点 x = -43.44329833984375
  实际位移约 = 0.18255 m
```

也就是说当前 UDP 轴值链路下，`vx=0.05` 的 1 秒实际前进距离约为 18.3cm，明显大于理论 5cm。因此继续用 `速度 * 时间` 的开环 1m 测试不可靠。

新增 ROS1 `/leg_odom` 闭环距离测试：

```text
robot_hardware/robot_hardware/robot_test_x30_udp_odom.cpp
```

该测试程序仍然通过 `DeepRoboticsX30::writeRobotVelocityCommand()` 发送 UDP 速度命令，但同时订阅 ROS1 `/leg_odom`，记录起点位置，实时计算平面位移，到达目标距离后自动发送零速度和 `stop_move`。

编译方式：

```bash
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

cd /home/ysc/robot_hardware/robot_hardware
rm -rf build_x30
mkdir build_x30
cd build_x30
cmake -DBUILD_X30_ONLY=ON -DBUILD_X30_ROS1_ODOM_TEST=ON ..
make -j$(nproc)
```

推荐先测 0.30m 闭环，不要直接 1m：

```bash
./robot_test_x30_udp_odom ../config.yaml zero
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15
```

参数含义：

```text
forward 方向
0.30    目标平面位移 m
0.02    发送速度 m/s
15      最大运行时间 s
```

如果 0.30m 闭环停止可靠，再逐步提高到：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 0.50 0.02 20
./robot_test_x30_udp_odom ../config.yaml forward 1.00 0.02 30
```

0.30m 闭环首次测试结果：

```text
命令：
  ./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15

起点：
  x = -44.5148
  y = 163.031

过程：
  distance_m = 0
  distance_m = 0.0532875
  distance_m = 0.148743
  distance_m = 0.235016

结果：
  reached = true
  final_distance_m = 0.300182
```

结论：

```text
1. /leg_odom 闭环距离停止有效。
2. 但这只证明“到距离能停”，不证明 writeRobotVelocityCommand(vx) 与真实速度一致。
3. 速度语义仍需要校验，否则上层导航输出 vx/vy/omega 时会失真。
```

因此 `robot_test_x30_udp_odom` 已继续增强，订阅 `/robot_velocity` 并输出速度统计：

```text
elapsed_s
avg_odom_speed_mps
speed_scale
avg_feedback_velocity_mps
last_feedback_velocity_mps
```

其中：

```text
avg_odom_speed_mps = /leg_odom 位移 / 实际运行时间
speed_scale = avg_odom_speed_mps / cmd_speed_mps
```

如果 `speed_scale` 明显大于 1，说明当前 UDP 轴值链路下，统一接口输入速度被放大；后续需要做速度映射校准，或在正式控制中基于反馈做闭环。

## 11. 2026-07-10 速度语义实测与规格书核对

本次重新编译带速度统计的 `/leg_odom` 闭环测试程序后，完成了一次前进 0.30m 测试：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15
```

实测输出：

```text
motion_command_source = navigation
目标距离 = 0.30 m
统一接口输入 cmd.vx = 0.02 m/s

reached = true
final_distance_m = 0.302643
elapsed_s = 1.85727
avg_odom_speed_mps = 0.16295
speed_scale = 8.14751
avg_feedback_velocity_mps = 0.231887
last_feedback_velocity_mps = 0.367154
```

结论：

```text
1. /leg_odom 闭环距离停止继续有效，0.30m 时实际停止在约 0.303m。
2. 当前输入 cmd.vx=0.02 m/s，但按 /leg_odom 统计的平均速度约为 0.163 m/s，约为输入的 8.15 倍。
3. /robot_velocity 的最后反馈为 0.367 m/s，与输入 0.02 m/s 也明显不一致。
4. 因此当前 UDP 手动/轴值链路尚不能直接作为“物理单位速度严格一致”的正式导航后端。
```

规格书第 12 页说明，前后轴值并非线性全范围，而是存在 `[-6553, 6553]` 摇杆死区：

```text
vx = (axis - 6553) / 26215 * 当前步态最大前进速度
```

当前 `DeepRoboticsX30::velocityToAxis()` 正是按这条反公式生成轴值。以当前配置
`max_vx=1.0` 和 `cmd.vx=0.02` 为例，发送轴值约为：

```text
axis = 6553 + 0.02 / 1.0 * 26215 = 7077
```

所以 `6553` 是规格书规定的轴值死区边界，不是代码人为把所有非零速度强制设为
`0.2 m/s`。本次异常的根因仍需继续确认，优先检查当前 RL/手动状态下 `0x31` 导航发送源
的实际执行语义，以及 `/leg_odom`、`/robot_velocity` 对该控制链路的反馈一致性。

后续速度校验原则：

```text
1. 暂不通过修改 max_vx/max_vy/max_omega 来硬凑实测速度；它们应保持为当前步态的真实上限。
2. 每次只测一个方向和一个小速度，记录 final_distance_m、elapsed_s、avg_odom_speed_mps、
   speed_scale、avg_feedback_velocity_mps。
3. 得到稳定的多点曲线后，再决定是否在 X30 后端引入明确命名的校准系数，或改用 ROS /cmd_vel
   原厂导航安全链路作为正式速度控制后端。
```

### 11.1 同输入复测与基本状态确认

在另一终端持续观察 `/robot_basic_state` 的同时，重复执行相同测试：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15
```

观察结果：

```text
/robot_basic_state:
  测试前、toggle 后、运动期间均为 data: 16

第二次距离测试：
  final_distance_m = 0.305870
  elapsed_s = 1.90780
  avg_odom_speed_mps = 0.160326
  speed_scale = 8.01628
  avg_feedback_velocity_mps = 0.276889
  last_feedback_velocity_mps = 0.367154
```

两次 `cmd.vx=0.02` 的结果对比：

| 项目 | 第一次 | 第二次 | 结论 |
|---|---:|---:|---|
| 平均里程计速度 | 0.162950 m/s | 0.160326 m/s | 差约 1.6%，结果可重复 |
| 速度倍率 | 8.14751 | 8.01628 | 当前 RL 状态下约为 8.08 倍 |
| 基本状态 | 16 | 16 | 未进入文档定义的 4 踏步状态 |

这说明：

```text
1. 当前 0x31 UDP 轴值链路在 RL 状态下的实际速度响应稳定，但与统一接口的 m/s 输入不一致。
2. 规格书第 12 页的轴值速度公式属于“踏步状态下的轴指令”；当前测试没有进入 basic_state=4，
   因而不能用该公式证明 RL 状态下的真实速度一定正确。
3. 旧测试输出“Expected: force stand -> stepping”不符合这次实测。测试工具已改为自动订阅
   /robot_basic_state，在 toggle 前后打印实际状态，并明确标注“不匹配文档踏步状态 4”。
4. 继续做 UDP 速度校验时，应固定在当前 RL 状态，使用 /leg_odom 做距离安全边界和速度统计，
   不应把当前约 8.08 倍的比例直接写入 max_vx 配置。
```

### 11.2 状态自动记录版第三次复测

编译状态自动记录版 `robot_test_x30_udp_odom` 后，再次运行：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15
```

程序自动观察到：

```text
Basic state before motion toggle: 16
Basic state after motion toggle: 16
Basic state before final motion toggle: 16
Basic state after final motion toggle: 16
```

第三次速度统计：

```text
final_distance_m = 0.303658
elapsed_s = 1.857630
avg_odom_speed_mps = 0.163465
speed_scale = 8.17323
avg_feedback_velocity_mps = 0.202901
last_feedback_velocity_mps = 0.367154
```

三次相同输入的汇总：

| 统一接口输入 | 平均里程计速度 | 速度倍率 |
|---:|---:|---:|
| `cmd.vx = 0.02 m/s` | 0.162247 m/s（三次平均） | 8.11234（三次平均） |

三次平均速度的最大差异约为 1.9%，说明当前结果具有较好的重复性。

接口规格书和应用手册已确认：

```text
basic_state = 16：RL 状态
gait_state = 32：L 行走步态
```

但公开文档没有提供 RL 状态下轴指令到物理速度的单独映射公式。规格书给出的轴值公式
明确写在“踏步状态下的轴指令”部分。因此当前测试结论是：

```text
1. UDP 轴值链路在本机 RL/L 行走状态下可以稳定驱动机器人。
2. /leg_odom 闭环可将单次位移限制在目标距离附近。
3. 当前不能声称 writeRobotVelocityCommand(cmd.vx) 的输入单位与 RL 实际速度单位一致。
4. 正式自主导航仍应优先走 ROS /cmd_vel -> 原厂地形图/安全层 -> /cmd_vel_corrected 链路。
5. 若继续研究 UDP 后端，需要在固定 RL 状态下完成 vx、vy、omega 多点实测曲线，
   再决定是否增加明确命名、可关闭的校准层。
```

### 11.3 低输入速度平台确认

在相同 RL/L 行走状态下，继续测试更低的前进输入：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.01 35
```

结果：

```text
basic_state = 16（测试前后均未改变）
final_distance_m = 0.304802
elapsed_s = 1.911440
avg_odom_speed_mps = 0.159462
speed_scale = 15.9462
avg_feedback_velocity_mps = 0.271970
last_feedback_velocity_mps = 0.353561
```

与 `cmd.vx=0.02` 的三次平均值对比：

| 统一接口输入 | 平均里程计速度 |
|---:|---:|
| `0.01 m/s` | `0.159462 m/s` |
| `0.02 m/s` | `0.162247 m/s` |

两者实际平均速度仅相差约 `1.7%`，但输入相差一倍。结论：

```text
1. 当前 RL UDP 链路的低输入区间不是线性比例关系。
2. 对本代码而言，cmd.vx=0.01 和 cmd.vx=0.02 分别会生成约 6815、7077 的轴值；
   按“踏步状态”规格书公式应对应不同物理速度，但 RL 实测几乎相同。
3. 当前行为接近“零速度 = 停止，任意小非零速度 = 约 0.16 m/s 的最小有效前进速度”。
4. 因此不应继续通过拟合一个固定 speed_scale 来修复 DeepRoboticsX30 UDP 后端，
   否则上层局部规划、避障和到点控制会失去连续速度调节能力。
5. UDP 轴值后端保留用于手动/RL 底层连通性与方向验证；正式自主导航速度接口应改走
   ROS /cmd_vel -> 原厂安全层 -> /cmd_vel_corrected。
```

### 11.4 最大前进输入 1 秒测试

为确认高轴值是否仍有速度响应，执行：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 1.00 1.00 1
```

本次不是“保证走 1m”，而是：目标距离上限为 1m，最大连续发速度时间为 1 秒。当前
`max_vx=1.0` 时，`cmd.vx=1.0` 会发出最大前进轴值 `32767`。

结果：

```text
reached = false
final_distance_m = 0.205242
elapsed_s = 1.003840
avg_odom_speed_mps = 0.204458
last_feedback_velocity_mps = 0.798993
basic_state = 16（测试前后均未改变）
```

说明：

```text
1. reached=false 符合预期：1 秒时间先到，尚未达到 1m 位移上限。
2. 最大输入下，/robot_velocity 的最后一帧已达到约 0.799m/s，说明高轴值确实会提高
   RL 的运动响应，不是所有非零输入都固定在 0.16m/s。
3. avg_odom_speed_mps=0.204m/s 是从起步到停止的整段平均值，包含起步加速过程，不能把它
   当作 cmd.vx=1.0 的稳态速度映射或速度倍率。
4. 当前可以确认 RL 低输入区间存在平台/非线性，高输入区间还存在明显加速过程；若要形成
   可用于控制的 UDP 速度模型，下一步应使用“预热 + 稳态采样 + 距离上限”的专用速度阶跃测试。
```

### 11.5 最大前进输入 1 秒复测

重复执行同一最大输入测试：

```bash
./robot_test_x30_udp_odom ../config.yaml forward 1.00 1.00 1
```

第二次结果：

```text
final_distance_m = 0.215832
elapsed_s = 1.004180
avg_odom_speed_mps = 0.214933
avg_feedback_velocity_mps = 0.225499
last_feedback_velocity_mps = 0.409998
```

与第一次最大输入测试对比：

| 指标 | 第一次 | 第二次 |
|---|---:|---:|
| 1 秒位移 | 0.205242 m | 0.215832 m |
| 平均里程计速度 | 0.204458 m/s | 0.214933 m/s |
| 最后一帧 `/robot_velocity` | 0.798993 m/s | 0.409998 m/s |

结论：

```text
1. 前 1 秒的平均位移结果接近，说明最大输入确实能持续驱动机器人前进。
2. 最后一帧速度反馈差异较大，不能把单帧 /robot_velocity 或 1 秒平均位移视为稳态最大速度。
3. 现有程序用于“位移上限 + 自动停车”是合格的；用于建立速度模型还缺少预热、稳定采样窗口、
   最大距离保护和多次统计。
4. 在专用速度阶跃测试完成前，不应继续用 1 秒最大输入测试推断或修改 X30 的速度映射。
```

## 12. 2026-07-10 速度一致性根因调查

已确认当前 `DeepRoboticsX30` 是摇杆轴值 UDP 后端，不是已验证的物理速度 UDP 后端。`max_vx=1.0` 与 L 行走步态规格一致，不是低速平台的主要原因。

完整根因、官方 SDK 调研、原厂 `udp_sender` 抓包流程和后续实现方案见：

```text
docs/X30_UDP速度一致性根因与原厂接口调研_20260710.md
```

本次还将两个 UDP 轴值测试程序的持续发送频率从 20Hz 修正为规格书要求的 50Hz。
