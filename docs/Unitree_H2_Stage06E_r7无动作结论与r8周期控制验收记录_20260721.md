# Unitree H2：Stage 06E r7 无动作结论与 r8 周期控制验收记录

日期：2026-07-21，Asia/Shanghai  
状态：r7 实机物理验收失败；r8 本地实现、编译与门禁合同测试已通过，等待 PC2 实机执行。

## 1. 本轮输入和结论

r7 在 PC2 上的原始日志：

```text
release=/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7
log=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06e_20260721_091811_4421.log
boot_id=0cf8dc4a-21fd-4f35-9c38-95ca907acef3
H2_LIVE_PULSE_RPC ret=0
COMMAND_RC[single_axis_motion]=0
fsm_id_before=601
fsm_id_after=601
physical_observation=NO_PHYSICAL_MOTION_OBSERVED
```

结论必须分层：

- `RobotHardwareInterface -> UnitreeH2 -> LocoClient::SetVelocity()` 软件调用链可达；
- 原厂 RPC 返回 0，前后 `FSM=601`，结束 `StopMove()` 返回 0；
- 没有观察到踏步、位移或方向响应，因此 Stage 06E 物理运动验收失败；
- r7 错误写出的 `stage06e.ok` 无效，不能作为后续阶段的父 gate。

## 2. r7 为什么可能完全不可见

r7 只调用一次 `writeRobotVelocityCommand()`，参数为 `0.08 m/s`，本地 200 ms
后立即调用 `StopMove()`。不考虑步态启动、斜坡和滤波时，理论位移也只有：

```text
0.08 m/s * 0.20 s = 0.016 m
```

宇树当前固定 SDK2 中，`SetVelocity()` 默认 duration 是 1 秒；官方 H2 高层示例使用
`0.5 m/s * 1 s`。官方没有公开 H2 的最小有效速度或最短步态启动时间，因此
“200 ms 未跨过步态启动或滤波阈值”只能作为当前工程推断，不能写成原厂硬性规定。

当前 `FSM=601` 已等于 SDK `Start()` 的目标 FSM；H2 高层 `sport` RPC 也不要求
进入 `L2+R2` 的底层调试模式，所以本轮不调用 `Start()`，也不切换到底层调试模式。

官方资料：

- <https://support.unitree.com/home/zh/H2_developer/rpc_routine>
- <https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/include/unitree/robot/h2/loco/h2_loco_client.hpp>
- <https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/h2/high_level/h2_loco_client_example.cpp>

## 3. r8 计划

不修改 `RobotHardwareInterface`、`RobotFactory` 或 `UnitreeH2` HAL 映射，不改用
ROS 2 topic，也不调用原厂 `Move(true)`。r8 模拟未来导航控制器的真实使用方式：

```text
上层控制周期 20 Hz
        -> writeRobotVelocityCommand(RobotVelocityCommand&)
        -> UnitreeH2::writeRobotVelocityCommand()
        -> 每命令 FSM=601 检查
        -> LocoClient::SetVelocity(vx, vy, omega, 0.300 s)
```

首个 profile：

| 项目 | r8 值 |
|---|---:|
| 轴 | `x-positive`，物理方向尚未标定 |
| 线速度 | `0.08 m/s` |
| 周期 | `20 Hz / 50 ms` |
| 总持续时间 | `1000 ms` |
| 非零抽象接口调用数 | `20` |
| 最大允许实测发送间隔 | `100 ms` |
| HAL watchdog | `150 ms` |
| SDK RPC timeout | `0.200 s` |
| 单条原厂命令 duration | `0.300 s` |
| 停止后零速观察 | `1000 ms` |

时序不变量：

```text
50 ms command period < 100 ms max gap < 150 ms watchdog < 300 ms vendor TTL
```

任一次抽象接口返回非 0、信号到达、发送间隔超过 100 ms、流持续时间越界或
FSM 不再是 601，立即禁止后续非零命令并执行 `StopMove()` 和零速度。watchdog 与
控制 RPC 共用 HAL mutex，所以它不是硬实时急停；0.300 秒原厂命令 TTL 和第二操作者
遥控器仍是独立保护层。

## 4. 实际修改

### 4.1 C++

- `robot_hardware/robot_hardware/include/unitree/unitree_h2_live_motion_plan.h`
  - stream 范围 `250..1000 ms`，只允许 50 ms 步进；
  - 固定 20 Hz、100 ms 最大间隔、150 ms watchdog、0.300 秒 vendor duration；
  - 速度边界保持 `linear <= 0.10 m/s`、`yaw <= 0.15 rad/s`。
- `robot_hardware/robot_hardware/robot_test_unitree_h2_live_motion.cpp`
  - `--pulse-ms` 改为 `--stream-ms`；
  - 使用 `steady_clock` 绝对时刻调度；
  - 所有非零命令只经 `writeRobotVelocityCommand()`；
  - 逐次记录 RPC 返回、延迟和发送间隔；
  - 完成、失败、超时或信号路径均执行停止。
- `robot_hardware/robot_hardware/tests/unitree_h2_live_motion_plan_test.cpp`
  - 覆盖默认 profile、最短流、速度/时长越界、步进和时序不变量。

未修改：

- `robot_hardware_interface.h`；
- `robot_factory.h` 的原有可见分配结构；
- `src/unitree/unitree_h2.cpp` 的 API 映射；
- ROS 2 topic 或 Docker 传感器链。

### 4.2 PC2 门禁

- `08_pc2_h2_single_axis_motion_gate.sh`
  - 授权短语绑定完整 profile：
    `RUN_STREAM_x-positive_L0.080_Y0.080_S1000_H20`；
  - 要求 `rpc_count == expected_rpc_count`；
  - 要求实测最大发送间隔不超过 100 ms；
  - 新增 `h2_pc2_axis_stream_observations.tsv`。
- `h2_pc2_hal_gate_common.sh`
  - Stage 06E gate 改为 stream schema；
  - `NO_PHYSICAL_MOTION_OBSERVED` 不能构成成功 gate。
- 成功人工确认改为 `BOUNDED_STREAM_OBSERVED_SAFE`，避免继续使用 r7 的
  `TINY_PULSE` 名称。

## 5. 本地验证

已经实际完成：

```text
UNITREE_H2_LIVE_MOTION_PLAN_OK
H2_GATE_SCHEMA_OFFLINE_OK
```

CLI 已使用固定 SDK2 头、r7 私有动态库和 Ubuntu 22.04 WSL 工具链编译并运行
print-only：

```text
H2_LIVE_PLAN axis=x-positive vx=0.080 vy=0.000 omega=0.000
linear_speed=0.080 yaw_speed=0.080 vendor_duration_s=0.300
stream_ms=1000 command_hz=20 command_period_ms=50 max_send_gap_ms=100
expected_rpc_count=20 watchdog_ms=150 sdk_timeout_s=0.200 expected_fsm=601
H2_LIVE_PRINT_PLAN_ONLY_NO_DDS
```

`--stream-ms 1001` 在 DDS 初始化前返回 64。SDK/CycloneDDS 头产生的旧式 variadic
macro 和 flexible-array warning 与前几版一致；编译、链接和上述合同测试成功。

## 6. r8 产物与离线验收

正式产物：

```text
path=D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8.tar.gz
size_bytes=8018363
sha256=ac51bd6544eaea8467cf9472aea74bc29dba1889671c50a20b26d1976284e0cd
manifest_file_sha256=1f85b7c62030bf06280884d9206b6bb2f516cc73273ab68bcce1e1b5678d4a82
parent_r7_sha256=3612e704a0472ba25a751146824f994e41e2c358bd12b2eaf95aae76e1abbebe
```

可重复构建和独立校验入口：

- `unitreeH2/docker/repackage_stage06e_bounded_stream_r8_without_docker.ps1`；
- `unitreeH2/docker/verify_stage06e_bounded_stream_r8_offline.ps1`。

独立 verifier 实际通过：

```text
H2_GATE_SCHEMA_OFFLINE_OK
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
UNITREE_H2_LIVE_MOTION_PLAN_OK
R8_NO_MOTION_NO_PROMOTION_OFFLINE_OK
R8_OFFLINE_VERIFY_OK
R8_OFFLINE_VERIFY_HOST_OK
```

同时验证了 manifest 全项、私有 `ldd`/RUNPATH、无 ROS 动态依赖、250 ms 合法下界、
默认 1000 ms/20 RPC，以及 249/275/1050 ms、重复参数和旧 `--pulse-ms` 都在
DDS 初始化前以代码 64 拒绝。

## 7. PC2 实机执行记录

### 7.1 第一次 r8 切换尝试

2026-07-21 10:03，PC2 执行 `cd ...20260721_r8` 时返回：

```text
-bash: cd: /home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8: No such file or directory
```

由于当前工作目录仍是 `...20260720_r7`，随后实际执行的是 r7 的 06C，而不是 r8：

```text
RELEASE=/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7
LOG=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_20260721_100341_17810.log
MANIFEST_SHA256=3f077089540e7c2ae9018dc87edbd166808a28dbb908e407d6e6bd52e854495e
H2_STAGE06C_GETTER_RPC_OK fsm_id=601 fsm_mode=0
```

这一步是 getter-only，只读成功，没有发送速度或状态切换命令；但它不能作为 r8 的
06C 证据。必须停止在这里，确认 r8 tar.gz 已上传、校验并解包后，再从 r8 目录重新
运行 06C。

### 7.2 后续边界

2026-07-21，r8 目录中的 06C 已重新执行成功，终端尾部证据为：

```text
PWD=/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8
H2_GETTER_ONLY_RPC_OK
COMMAND_RC[getter_audit]=0
GETTER_AUDIT_GATE_OK fsm_id=601 fsm_mode=0 mode_value_observation_only=1
GATE_WRITTEN=/home/unitree/p2_unitreeH2/build/h2_control_gate_state/stage06c.ok
H2_STAGE06C_GETTER_RPC_OK fsm_id=601 fsm_mode=0
```

因此 r8 的 getter-only 门禁已通过，可以进入 06D 的受保护零速度和 `StopMove()`
测试；该结论仍不包含任何非零运动或物理位移证明。

当前可以继续执行同一 boot、同一 r8 release 下的 06D。若机器人重启、release
变化或重新执行 06C，则下游 gate 必须重新生成；不能复用 r7 错误生成的
`stage06e.ok`。若 r8 仍无动作，必须输入 `NO_PHYSICAL_MOTION_OBSERVED`；脚本应
返回 52 且不生成 `stage06e.ok`。只有实际观察到物理动作且未发生异常，才输入
`BOUNDED_STREAM_OBSERVED_SAFE`。

## 8. r8 Stage 06E 实机最终结果

2026-07-21，在同一 r8 release 下，06C getter-only 和 06D 受保护零速度门禁已先后
通过。随后执行 `x-positive` 的一次性受限速度流：

```text
vx=0.080 m/s
vy=0
omega=0
stream_ms=1000
command_hz=20
expected_rpc_count=20
vendor_duration_s=0.300
watchdog_ms=150
sdk_timeout_s=0.200
expected_fsm=601
```

实际 RPC 和时序证据：

```text
rpc_count=20
20/20 SetVelocity ret=0
max_observed_send_gap_ms=52
max_rpc_latency_ms=1
stream_elapsed_ms=1000
pre_stream_fsm_id=601
final_fsm_id=601
stream-complete StopMove ret=0, zero_velocity ret=0
scope-exit StopMove ret=0, zero_velocity ret=0
COMMAND_RC[single_axis_motion]=0
```

操作员实际观察并输入：

```text
NO_PHYSICAL_MOTION_OBSERVED
```

脚本最终结果：

```text
H2_STAGE06E_NO_MOTION_OBSERVED axis=x-positive linear_speed=0.080 yaw_speed=0.080 stream_ms=1000 command_hz=20 rpc_count=20 observation=NO_PHYSICAL_MOTION_OBSERVED stage06e_gate_written=0
H2_GATE_FAILED=PHYSICAL_MOTION_NOT_OBSERVED_NO_STAGE06E_GATE
```

结论：r8 已证明抽象接口到 `LocoClient::SetVelocity()` 的 20 Hz RPC 调用、返回值、
时序、FSM 复查和停止清理都符合计划；但机器人没有产生可见物理运动，所以 Stage 06E
未通过，`stage06e.ok` 没有生成。厂商返回 0 只能解释为 RPC 成功，不能解释为运动控制器
已经采纳命令或机器人已经位移。

附件日志 SHA256：

```text
99e157c7d8a78a62a3fa8a57944259110e12bc3f702c759e60102c211f731442
```

下一步不直接放大脉冲。先在新的状态动作门禁内记录 `Start()` 前后
`GetFsmId/GetFsmMode/GetBalanceMode` 和状态收敛，再将宇树原始 H2 高层 RPC 示例与
本项目 HAL 做同状态、同参数 A/B 测试，并排查交付固件的命令仲裁、速度死区和保护约束。
