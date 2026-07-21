# Unitree H2：Stage 06E r7 可配置脉冲准备与离线验收记录

日期：2026-07-20（初始构建），2026-07-21（实机复测），Asia/Shanghai  
状态：r7 本地构建、离线验收和 PC2 实机 RPC 调用已完成；`SetVelocity`
返回 0，但未观察到任何物理动作，因此 Stage 06E 物理运动验收未通过。

## 1. 本轮目标

r6 已在 H2 实机上完成一次 `x-positive` 非零速度 RPC，但使用的 `0.050 m/s +
120 ms` 脉冲几乎不可见，现场只能记录 `direction unclear`。本轮目标是在不手工修改
脚本、不改变 `RobotHardwareInterface` 和 `UnitreeH2` HAL 的前提下，让每次 06E 调用
可以显式选择单轴速度和脉冲时长，同时继续保留：

- `FSM == 601` 初始化与每命令门禁；
- 06C getter、06D 零速度/`StopMove()` 父 gate；
- 一次只允许一个未标定轴；
- 一次性授权文件、5 秒倒计时、信号停止和 watchdog；
- 最后时刻人工确认、第二操作者遥控器待命和结束 `StopMove()`；
- 不调用 `Start`、`StandUp`、`Damp`、`Squat`、`Sit`，不发布 `rt/lowcmd`。

本轮继续使用 Stage 06E gate 名称。它是“r7 可配置 06E 复测”，不是新增的 Stage
06F；Stage 06F 留给六轴方向映射完成后的下一阶段控制验收。

## 2. 计划

1. 把速度和脉冲时长变成命令行参数，并设置编译期上下限；
2. 让 print-only 计划、实机命令、最终人工确认、gate 和 TSV 使用同一组实际参数；
3. 保持 r6 默认 profile 不变；
4. 只重编 live-motion CLI 和 plan test，复用 r6 已实机验证的 HAL 动态库；
5. 校验 manifest、私有动态依赖、无 ROS 依赖、合同测试和越界拒绝；
6. 生成完整 r7 tar.gz 与相对文件名 SHA256，PC2 只接收完整包。

## 3. 审计中发现并修复的问题

中断前的半成品把 SDK duration 写成 `(pulse_ms + 200) / 1000`。这会产生：

```text
120 ms -> 0.320 s
200 ms -> 0.400 s
```

但 `UnitreeH2` 的硬上限是 `velocity_command_duration_s <= 0.30 s`。若直接上机，
`initRobotHardware()` 会返回无效配置，默认和建议 profile 都无法运行。

最终修正为：

```text
watchdog_ms      = pulse_ms + 60
vendor_duration  = max(0.25, (pulse_ms + 100) / 1000)
max pulse_ms     = 200
```

因此：

| profile | 本地脉冲 | watchdog | SDK duration |
|---|---:|---:|---:|
| r6 兼容默认 | 120 ms | 180 ms | 0.250 s |
| r7 首次建议 | 200 ms | 260 ms | 0.300 s |

两组都满足 `pulse < watchdog < SDK duration <= 0.30 s`，没有提高 HAL 的硬上限。

## 4. 实际修改

### 4.1 C++ 计划与 CLI

- `robot_hardware/robot_hardware/include/unitree/unitree_h2_live_motion_plan.h`
  - 新增运行时 profile；
  - 线速度范围 `0.01..0.10 m/s`；
  - 角速度范围 `0.01..0.15 rad/s`；
  - 脉冲范围 `50..200 ms`；
  - 速度只允许 `0.001` 步进，避免实际值与三位小数日志不一致；
  - 检查 finite、单轴、watchdog 顺序和 `0.30 s` duration 上限。
- `robot_hardware/robot_hardware/robot_test_unitree_h2_live_motion.cpp`
  - 新增 `--linear-speed`、`--yaw-speed`、`--pulse-ms`；
  - 重复参数拒绝；
  - 实际 profile 写入进程内 YAML override，再通过
    `RobotHardwareInterface::writeRobotVelocityCommand()` 调用 HAL。
- `robot_hardware/robot_hardware/tests/unitree_h2_live_motion_plan_test.cpp`
  - 直接断言 r6 默认时序；
  - 断言 `0.08 + 200 ms` 计划；
  - 覆盖上限、下限、精度、单轴和 duration 拒绝。

### 4.2 PC2 gate 与日志

- `unitreeH2/remote/08_pc2_h2_single_axis_motion_gate.sh`
  - 参数透传给 print-only 和 live 两次调用；
  - 最终确认短语绑定轴、线速度、角速度和脉冲，例如
    `RUN_x-positive_L0.080_Y0.080_P200`；
  - 最终确认前重新打印完整 profile；
  - 新增 `h2_pc2_axis_profile_observations.tsv`，保留每次 profile 和观察结果。
- `unitreeH2/remote/h2_pc2_hal_gate_common.sh`
  - 06E gate 增加 profile 字段；
  - 校验数值范围以及 `pulse < watchdog < duration`。
- `unitreeH2/remote/tests/test_h2_gate_schema_offline.sh`
  - 增加格式、越界和时序关系拒绝用例。

## 5. r7 构建方法与不变边界

构建脚本：

```text
D:\Desktop\RoboDog\unitreeH2\docker\repackage_stage06e_configurable_r7_without_docker.ps1
```

该脚本以 r6 为父包，仅重编：

```text
bin/robot_test_unitree_h2_live_motion
bin/unitree_h2_live_motion_plan_test
```

以下内容已与 r6 逐字节比较并保持不变：

```text
lib/
config/
bin/robot_test_unitree_h2
bin/unitree_h2_factory_contract_test
bin/unitree_h2_direct_api_contract_test
scripts/06_pc2_h2_getters_rpc_gate.sh
scripts/07_pc2_h2_zero_stop_gate.sh
meta/image-id.txt
meta/sdk2-commit.txt
meta/symlinks.txt
```

固定构建输入：

```text
r6 sha256=0200c41efcd8840103ee3f97b50fc1a759f12c20db42061fe5883186a7cadb64
SDK2 commit=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
SDK2 include tree sha256=c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b
yaml-cpp dev deb sha256=28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a
```

`libyaml-cpp-dev_0.7.0+dfsg-8build1_amd64.deb` 已缓存到
`unitreeH2/downloads/`，打包脚本只解出头文件，不在 Windows、WSL 或 PC2 安装软件，
也不在 PC2 使用其旧 SDK archive 重新编译。

## 6. 产物与离线验收结果

产物：

```text
D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz
size=8009547 bytes
sha256=3612e704a0472ba25a751146824f994e41e2c358bd12b2eaf95aae76e1abbebe
manifest file sha256=5e3ac3c6abeddb1a5631f082adb1b043bf29b8361bc9c2397e5dc7f13fd77941
```

独立复核脚本：

```text
D:\Desktop\RoboDog\unitreeH2\docker\verify_stage06e_configurable_r7_offline.ps1
```

实际通过标记：

```text
H2_GATE_SCHEMA_OFFLINE_OK
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
UNITREE_H2_LIVE_MOTION_PLAN_OK
R7_CONFIGURABLE_PROFILE_REPACKAGE_OK
R7_CONFIGURABLE_PROFILE_REPACKAGE_HOST_OK
R7_OFFLINE_VERIFY_OK
R7_OFFLINE_VERIFY_HOST_OK
```

推荐 profile 的实际 print-only 输出：

```text
H2_LIVE_PLAN axis=x-positive vx=0.080 vy=0.000 omega=0.000 linear_speed=0.080 yaw_speed=0.080 vendor_duration_s=0.300 local_pulse_ms=200 watchdog_ms=260 expected_fsm=601
H2_LIVE_PRINT_PLAN_ONLY_NO_DDS
```

`ldd` 全部解析到 r7 包内的 `librobot_hardware`、`yaml-cpp`、Zsibot、`ddsc` 和
`ddscxx`，没有 `rcl`、`rmw` 或 `ros` 动态依赖。构建输出中的 warning 来自固定
SDK2/CycloneDDS 头的旧式 variadic macro 和 flexible array；编译、链接和合同测试均返回
0。

## 7. 2026-07-21 r7 实机结果

实际运行：

```text
release=/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7
log=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06e_20260721_091811_4421.log
boot_id=0cf8dc4a-21fd-4f35-9c38-95ca907acef3
axis=x-positive
vx=0.080 m/s
local_pulse=200 ms
vendor_duration=0.300 s
watchdog=260 ms
```

软件侧实际证据：

```text
H2_LIVE_PRE_PULSE_STATE fsm_id=601 fsm_ret=0
H2_LIVE_PULSE_RPC ret=0
H2_LIVE_STOP reason=pulse-complete stop_move_ret=0 zero_velocity_ret=0
H2_LIVE_FINAL_STATE fsm_id=601 fsm_ret=0 stop_ret=0
COMMAND_RC[single_axis_motion]=0
```

现场观察为：

```text
NO_PHYSICAL_MOTION_OBSERVED
```

结论分层：

- 已证明 `RobotHardwareInterface -> UnitreeH2 -> LocoClient::SetVelocity()` 调用路径可达，
  并且原厂 RPC 返回 0；
- 已证明结束 `StopMove()` 和前后 `FSM=601` 门禁正常；
- 没有证明机器人产生了位移、踏步或轴方向响应；
- `0.08 m/s * 0.20 s` 理论目标仅 `0.016 m`，且 200 ms 后立即
  `StopMove()`，可能未跨过步态启动、速度斜坡或滤波阈值；这是当前推断，
  不是宇树已公开的硬性阈值。

r7 还暴露了一个门禁语义缺口：脚本只要观察字符串非空，即使内容是
`NO_PHYSICAL_MOTION_OBSERVED`，仍会写入 `stage06e.ok` 并输出
`H2_STAGE06E_SINGLE_AXIS_PULSE_OK`。该 gate 只能视为“RPC/Stop 调用记录”，
不能视为物理运动成功证据。后续包必须在“无动作”时拒绝写入成功 gate。

2026-07-21 本地工作区已修复该语义：`NO_PHYSICAL_MOTION_OBSERVED` 会记录原始
观察值、输出 `stage06e_gate_written=0` 并以非零代码结束，不再询问后续“成功”
确认，也不写 `stage06e.ok`。该修复不会反向修改已经上传的 r7 历史包；PC2 上由
r7 错误生成的 gate 必须先改名归档，不能作为后续阶段的父 gate。

宇树官方 H2 高层 RPC 示例使用 `0.5 m/s * 1 s`，SDK 中非连续
`Move()` 也会转成 `SetVelocity(..., 1.f)`。官方同时明确高层 `sport`
RPC 例程无需进入 `L2+R2` 底层调试模式；当前 `FSM=601` 已等价于
SDK `Start()` 的目标 FSM，所以不重复调用 `Start()`。

## 8. 尚未完成

- r7 已上传 PC2，已重新执行 06C、06D 和 06E RPC；
- Stage 06E 物理运动验收仍未通过；
- `x-positive` 的物理方向仍未标定；
- `writeActionCommand()` 的状态切换动作仍未做实机测试；
- Stage 06B 生产状态源、导航传感器驱动、算法和正式 `h2_runtime` 仍未完成。

## 9. 原始上机命令（已执行，仅留档）

Windows PowerShell：

```powershell
scp `
  "D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz" `
  "D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz.sha256" `
  unitree@192.168.123.162:/home/unitree/p2_unitreeH2/build/
```

PC2 首先只校验、解包并运行 06C：

```bash
cd /home/unitree/p2_unitreeH2/build
sha256sum --check --strict \
  unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz.sha256

test ! -e unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7 || {
  echo "r7 release directory already exists; stop"
  exit 1
}

tar -xzf unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz
cd unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7
bash scripts/06_pc2_h2_getters_rpc_gate.sh
```

只有 06C 输出 `H2_STAGE06C_GETTER_RPC_OK`，且现场物理门禁全部重新确认后，才依次执行：

```bash
bash scripts/07_pc2_h2_zero_stop_gate.sh

bash scripts/08_pc2_h2_single_axis_motion_gate.sh \
  x-positive \
  --linear-speed 0.08 \
  --yaw-speed 0.08 \
  --pulse-ms 200
```

06E 最后时刻将要求输入：

```text
RUN_x-positive_L0.080_Y0.080_P200
```

如果观察到意外动作，不输入 `TINY_PULSE_OBSERVED_SAFE`，立即使用现场原厂停止/阻尼
流程并结束测试。每一步完成后回传完整终端输出和 `/home/unitree/p2_unitreeH2/logs/`
中的对应原始日志。
