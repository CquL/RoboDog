# X30 robot_hardware UDP 方向测试简明命令

更新时间：2026-07-08  
机器狗路径：`/home/ysc/robot_hardware/robot_hardware`

## 1. 本地传输

```powershell
cd D:\Desktop\RoboDog
scp .\robot_hardware_x30_udp_transfer.tar.gz ysc@192.168.1.106:/home/ysc/
```

## 2. 机器狗解压

以下命令在机器狗宿主机执行，不是在 Docker 内：

```bash
cd /home/ysc
mv robot_hardware robot_hardware.bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
tar -xzf robot_hardware_x30_udp_transfer.tar.gz
cd /home/ysc/robot_hardware/robot_hardware
```

## 3. 编译

普通 UDP 方向/短脉冲测试：

```bash
rm -rf build_x30
mkdir build_x30
cd build_x30
cmake -DBUILD_X30_ONLY=ON ..
make -j$(nproc)
```

如果要编译 `/leg_odom` 闭环距离测试，需要 ROS1 环境，并额外打开开关：

```bash
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

rm -rf build_x30
mkdir build_x30
cd build_x30
cmake -DBUILD_X30_ONLY=ON -DBUILD_X30_ROS1_ODOM_TEST=ON ..
make -j$(nproc)
```

## 4. 测试顺序

先看模式：

```bash
./robot_test_x30_udp ../config.yaml list
```

先跑零速度，确认不误动：

```bash
./robot_test_x30_udp ../config.yaml zero
```

逐个方向测试：

```bash
./robot_test_x30_udp ../config.yaml forward
./robot_test_x30_udp ../config.yaml backward
./robot_test_x30_udp ../config.yaml left
./robot_test_x30_udp ../config.yaml right
./robot_test_x30_udp ../config.yaml yaw_left
./robot_test_x30_udp ../config.yaml yaw_right
```

如果 1m 开环测试明显超距，先做 1 秒短脉冲标定：

```bash
./robot_test_x30_udp ../config.yaml forward_calib
./robot_test_x30_udp ../config.yaml backward_calib
./robot_test_x30_udp ../config.yaml left_calib
./robot_test_x30_udp ../config.yaml right_calib
```

如果要测试“到达指定里程计距离后自动停”，使用 odom 闭环测试：

```bash
./robot_test_x30_udp_odom ../config.yaml zero
./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15
```

建议保存日志：

```bash
mkdir -p ~/x30_odom_test_logs
stamp=$(date +%Y%m%d_%H%M%S)

./robot_test_x30_udp_odom ../config.yaml forward 0.30 0.02 15 \
  | tee ~/x30_odom_test_logs/${stamp}_forward_030m.log
```

参数含义：

```text
forward          方向，可选 forward/backward/left/right
0.30             目标平面位移，单位 m
0.02             发送给 writeRobotVelocityCommand 的速度，单位 m/s
15               最大运行时间，单位 s
```

重点查看最后一行：

```text
reached=true/false
final_distance_m=...
elapsed_s=...
avg_odom_speed_mps=...
speed_scale=...
avg_feedback_velocity_mps=...
last_feedback_velocity_mps=...
```

含义：

```text
avg_odom_speed_mps       /leg_odom 位移除以实际运行时间得到的平均速度
speed_scale              avg_odom_speed_mps / cmd_speed_mps，用于判断命令速度被放大多少
avg_feedback_velocity_mps /robot_velocity 在测试方向上的平均反馈速度
```

1m 线性距离测试：

```bash
./robot_test_x30_udp ../config.yaml forward_1m
./robot_test_x30_udp ../config.yaml backward_1m
./robot_test_x30_udp ../config.yaml left_1m
./robot_test_x30_udp ../config.yaml right_1m
```

全部单方向确认后，再运行：

```bash
./robot_test_x30_udp ../config.yaml all_safe
```

全部 1m 单方向确认后，再运行：

```bash
./robot_test_x30_udp ../config.yaml all_1m
```

## 5. 模式含义

```text
zero       只初始化和发零速度，不运动
forward    小速度前进
backward   小速度后退
left       小速度左移
right      小速度右移
yaw_left   小角速度左转
yaw_right  小角速度右转
all_safe   顺序执行所有方向测试
forward_calib  1 秒前进短脉冲标定
backward_calib 1 秒后退短脉冲标定
left_calib     1 秒左移短脉冲标定
right_calib    1 秒右移短脉冲标定
all_calib      顺序执行所有短脉冲标定
forward_1m 理论前进 1m
backward_1m 理论后退 1m
left_1m    理论左移 1m
right_1m   理论右移 1m
all_1m     顺序执行所有 1m 线性距离测试
```

odom 闭环测试单独由 `robot_test_x30_udp_odom` 提供：

```text
zero      初始化并发送零速度
forward   前进，到 /leg_odom 平面位移达到目标后停止
backward  后退，到 /leg_odom 平面位移达到目标后停止
left      左移，到 /leg_odom 平面位移达到目标后停止
right     右移，到 /leg_odom 平面位移达到目标后停止
```

1m 测试当前参数：

```text
vx = 0.10 m/s
vy = 0.10 m/s
持续时间 = 10 秒
理论距离 = 1 m
```

短脉冲标定当前参数：

```text
vx = 0.05 m/s
vy = 0.05 m/s
持续时间 = 1 秒
理论距离 = 0.05 m
```

## 6. 安全要求

```text
1. 先 zero，再单方向。
2. 不要一开始跑 all_safe 或 all_1m。
3. 手柄/APP 必须在手边，随时接管。
4. 测试区域保持空旷。
5. 每次观察方向是否正确、是否自动停止。
6. 1m 测试必须在短方向和短脉冲标定确认后再运行。
7. 更推荐用 robot_test_x30_udp_odom 做闭环距离测试，不继续盲目调 forward_1m。
```
