# Unitree H2 Docker 运动接口测试

## 1. 当前结论

当前 Docker 镜像：

```text
unitree_h2:amd64-runtime-candidate
Windows Docker Desktop OCI index ID:
sha256:f7cd06b3d28d90b68cffda3754050203a1cc438207ebf575cc090ad40b6e7d3c
H2 PC2 Docker Engine 24 amd64 image config ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
```

镜像构建时使用了：

```text
D:\Desktop\RoboDog\robot_hardware\robot_hardware
```

最终镜像没有保留完整源码目录，但已经安装了编译产物：

```text
/opt/robodog/bin/robot_test_unitree_h2
/opt/robodog/lib/librobot_hardware.so
/opt/robodog/config/unitree_h2_container_safe.yaml
/opt/robodog/config/unitree_h2_container_motion.yaml
```

统一测试程序的调用链为：

```text
robot_test_unitree_h2
  -> RobotFactory::RobotAllocate()
  -> RobotHardwareInterface
  -> UnitreeH2
  -> Unitree SDK2 LocoClient
```

对应三个抽象接口：

```text
initRobotHardware()
writeRobotVelocityCommand()
writeActionCommand()
```

速度测试会以 20 Hz 调用 `writeRobotVelocityCommand()`，结束时调用
`writeActionCommand(stop_move)` 并补发零速度。

## 2. 当前代码审计结果

`robot_test_unitree_h2.cpp` 可以用于已经人工进入 `FSM=601` 后的短时单轴运动测试，具备：

- 必须显式传入 `--execute`；
- 初始化及每条非零速度命令都检查 FSM；
- 速度和持续时间限制；
- 速度 watchdog；
- `SIGINT`、`SIGTERM` 收尾；
- 测试结束自动 `StopMove` 和零速度；
- 命令返回值、发送次数及测试前后 FSM 输出。

当前限制：

- 返回值 `0` 只证明 SDK/HAL/RPC 成功，不等于机器人必然产生物理位移；
- 没有里程计闭环验证；
- `vy`、`omega` 的正负物理方向还没有实机标定；
- 不允许同时运行两个速度测试进程；
- 本轮不测试 `stand_up`、`start`、`damp`、`squat`、`sit` 等状态动作。

仓库中的以下配置已经与适配器硬门禁同步：

```text
D:\Desktop\RoboDog\robot_hardware\robot_hardware\config\unitree_h2.yaml
```

当前项目允许上限为 `max_vx: 1.00`、`max_omega: 0.70`。这些是项目侧上限，
不是宇树公布的 H2 官方最大值。Docker 实机运动使用：

```text
/opt/robodog/config/unitree_h2_container_motion.yaml
```

只读接收与初始化继续使用 `unitree_h2_container_safe.yaml`，其运动开关保持关闭。

## 3. 实机前提

执行任何非零速度前必须满足：

- H2 位于平整、空旷区域；
- 机器人运动范围内无人和障碍物；
- 第二操作员持原厂遥控器，随时可以停止或进入阻尼；
- 人工选择“常规运控 1”；
- 查询得到 `FSM=601`；
- 遥控器摇杆保持中立；
- 没有其他速度测试或算法进程正在发送控制命令；
- 不停止或修改原厂运动服务；
- 每条运动命令单独执行，禁止一次性串联运行。

异常运动时优先使用原厂遥控器停止或进入阻尼，不等待终端命令。

## 4. 确认并进入容器

以下命令在 H2 PC2 宿主机执行：

```bash
sudo docker ps --format \
  'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'
```

如果容器名称是 `unitree-h2-imu-bridge`：

```bash
sudo docker inspect \
  --format 'image={{.Config.Image}} network={{.HostConfig.NetworkMode}} readonly={{.HostConfig.ReadonlyRootfs}}' \
  unitree-h2-imu-bridge
```

必须确认：

```text
image=unitree_h2:amd64-runtime-candidate
network=host
```

进入容器：

```bash
sudo docker exec -it unitree-h2-imu-bridge bash
```

如果 `docker ps` 显示的是其他容器名称，应把上述名称替换为实际名称。

## 5. 容器内检查

```bash
test -f /.dockerenv && echo IN_DOCKER

test -x /opt/robodog/bin/robot_test_unitree_h2 &&
test -f /opt/robodog/lib/librobot_hardware.so &&
test -f /opt/robodog/config/unitree_h2_container_safe.yaml &&
test -f /opt/robodog/config/unitree_h2_container_motion.yaml &&
echo H2_HAL_ARTIFACTS_OK

pgrep -af robot_test_unitree_h2 || true
```

若 `pgrep` 显示另一个运动测试仍在运行，停止本次测试。

设置命令变量：

```bash
BIN=/opt/robodog/bin/robot_test_unitree_h2
SAFE=/opt/robodog/config/unitree_h2_container_safe.yaml
MOTION=/opt/robodog/config/unitree_h2_container_motion.yaml
```

## 6. 只读 FSM 检查

```bash
"$BIN" \
  --config "$SAFE" \
  --getter-audit

echo "RC=$?"
```

必须出现：

```text
fsm_id=601
H2_GETTER_ONLY_RPC_OK
RC=0
```

若 FSM 不是 `601`，停止测试，不执行后面的命令。

## 7. 零速度停止检查

```bash
"$BIN" \
  --config "$SAFE" \
  --zero-stop \
  --execute

echo "RC=$?"
```

必须出现：

```text
H2_ZERO_STOP_RPC_OK fsm_id=601
RC=0
```

## 8. Docker 内前进测试

该参数已经在 PC2 原生 r11 中验证为向前移动，本次用于确认 Docker 路径：

```bash
"$BIN" \
  --config "$MOTION" \
  --velocity \
  --vx 0.50 \
  --vy 0 \
  --omega 0 \
  --duration-ms 1000 \
  --execute

echo "RC=$?"
```

预期软件结果：

```text
H2_HAL_STOP action_ret=0 zero_ret=0
H2_HAL_RESULT command_ret=0 stop_ret=0 sent_count=20 expected_count=20 fsm_after_ret=0 fsm_after=601 interrupted=0
RC=0
```

同时由现场观察员记录：

```text
实际是否向前：
命令结束后是否停止：
是否抖动或异常：
是否使用遥控器干预：
```

## 9. Docker 内转向测试

正负方向还没有标定，因此第一次只记录为 `yaw-positive` 和
`yaw-negative`，不要预先写成左转或右转。

先执行较小的正向角速度：

```bash
"$BIN" \
  --config "$MOTION" \
  --velocity \
  --vx 0 \
  --vy 0 \
  --omega 0.35 \
  --duration-ms 3000 \
  --execute

echo "RC=$?"
```

确认机器人已经停止且没有异常后，再测试反方向：

```bash
"$BIN" \
  --config "$MOTION" \
  --velocity \
  --vx 0 \
  --vy 0 \
  --omega -0.15 \
  --duration-ms 1000 \
  --execute

echo "RC=$?"
```

记录：

```text
omega=+0.15 的实际方向：
omega=-0.15 的实际方向：
是否产生可见转向：
命令结束后是否停止：
是否抖动或异常：
是否使用遥控器干预：
```

如果 RPC 返回 `0` 但没有可见转向，先记录“无可见动作”，不要连续重试或直接跳到
项目上限。当前项目允许的角速度上限是 `0.70 rad/s`，但尚未完成该值的实机验证。

## 10. 手动软件停止

正常情况下每次测试都会自动停止。需要再次发送软件停止时：

```bash
"$BIN" \
  --config "$SAFE" \
  --zero-stop \
  --execute
```

软件停止不能替代现场遥控器或物理安全措施。

## 11. 退出容器

```bash
exit
```

不要执行 `docker rm -f`，除非明确需要停止并删除当前 IMU Bridge 容器。

## 12. 测试记录

### 已完成

```text
2026-07-22：
PC2 原生 r11，HAL 抽象接口，vx=+0.50 m/s，1000 ms。
软件结果：20/20 次命令成功，StopMove 和零速度成功，FSM 前后均为 601。
物理结果：机器人向前移动并停止。
日志：unitreeH2/remote/h2_pc2_velocity_probe_hal_20260722_113813_37555.log

2026-07-28：
Docker 内双 IMU Bridge 已发布 /h2/imu/pelvis 和 /h2/imu/torso，
两个话题均已读取到有效 sensor_msgs/msg/Imu 样本。

2026-07-28，Docker 内首次 yaw-negative 测试：
参数：vx=0，vy=0，omega=-0.15 rad/s，duration=1000 ms。
软件结果：初始化成功，FSM 前后均为 601；20/20 次速度 RPC 返回 0；
StopMove action_ret=0，zero_ret=0，最终 RC=0。
物理结果：机器人没有可见转向动作。
结论：Docker -> RobotHardwareInterface -> UnitreeH2 -> LocoClient 软件链路成功，
但 -0.15 rad/s 尚未越过当前实机可见转向阈值。该记录发生时项目上限还是
max_omega=0.30 rad/s；2026-07-28 后项目允许上限调整为 0.70 rad/s，但尚未实机验证。

2026-07-28，项目速度包络与 Docker 镜像离线更新：
项目硬门禁及显式运动配置调整为 max_vx=1.00 m/s、max_omega=0.70 rad/s；
max_vy=0.10 m/s、SDK duration=0.30 s 和只读安全配置保持不变。
robot_hardware CTest 7/7 通过，镜像内 HAL、IMU 映射、ROS 2 IMU 端点及越界拒绝
全部通过。新镜像 ID 为
sha256:f7cd06b3d28d90b68cffda3754050203a1cc438207ebf575cc090ad40b6e7d3c。
本轮未连接 H2、未发送运动命令，1.00/0.70 的物理效果仍待分档实机验证。

2026-07-28，H2 PC2 新镜像部署验收：
容器 unitree-h2-imu-bridge 状态为 Up；容器 .Image 与 PC2 镜像标签 .Id 均为
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451；
容器内 unitree_h2_container_motion.yaml 已读取到 max_vx=1.00、max_omega=0.70。
结论：当前容器已绑定新镜像，新的速度包络配置已进入镜像。
本项没有发送非零运动命令。
```

### 本轮待填写

```text
测试时间：
镜像标签：
镜像 ID：
容器名称：
network_mode：
DDS 网卡/domain：
初始 FSM：

前进 vx/vy/omega：
前进 command_ret：
前进 stop_ret：
前进 sent_count/expected_count：
前进结束 FSM：
前进物理结果：

yaw-positive 参数：
yaw-positive 软件结果：
yaw-positive 物理方向：

yaw-negative 参数：
yaw-negative 软件结果：
yaw-negative 物理方向：

是否均在命令结束后停止：
是否出现抖动或异常：
是否使用遥控器干预：
操作员：
观察员：
终端日志路径：
```
