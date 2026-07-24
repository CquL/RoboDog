# Unitree H2 r9 简化速度 A/B 诊断与无动作定位记录

日期：2026-07-22

## 本轮目标

1. 审计 `D:\Desktop\robodog_h2_r9_patch` 是否已覆盖当前工程；
2. 区分正式 Stage 06E 验收脚本和日常底层速度调试入口；
3. 对“RPC 返回 0，但 H2 没有可见运动”建立原厂 SDK 与 RoboDog HAL 的同参数 A/B 测试；
4. 生成可上传到 H2 PC2 的完整 r9 amd64 包。

## 实际审计结果

- r9 patch 中 8 个目标文件均已进入当前工程；两处 SHA256 不同仅来自 LF/CRLF
  换行差异，文本内容一致。
- 当前 `unitreeH2/runtime_bundle` 在本轮前最高只有 r8，所以只修改 Windows
  源码不会改变 H2 上已经解压并执行的 r8 二进制。
- r8 实机证据仍是：`FSM=601`，20/20 次 `SetVelocity` 返回 0，停止返回 0，
  但没有观察到物理运动。返回 0 只能证明 `sport` RPC 接收请求，不能证明运动策略
  已采纳速度。
- r9 原有 Stage 06E 仍限制线速度不超过 `0.10 m/s`。宇树 H2 官方 RPC 例程
  使用单次 `SetVelocity(0.5, 0, 0, 1)` 演示。官方没有公开说明有效速度死区，
  因此当前不能把无动作直接归因于 HAL，也不能直接认定是速度过小。
- 官方 H2 RPC 例程明确写明高层 LocoClient 运行无需进入调试模式；调试模式用于
  底层例程。高层速度诊断应保持 `MotionSwitcher form=0 name=ai`，同时要求
  `FSM=601`。

## 为什么旧测试需要多次输入

旧的 `08_pc2_h2_single_axis_motion_gate.sh` 是一次正式实机验收流程，不是应用层
控制接口。长确认串分别用于物理场地确认、一次性授权、方向观察和成功 gate 留痕，
防止误把 RPC 返回 0 写成“机器人已运动”。这些步骤不应该放进导航控制循环，也
不应该成为每次速度调用的交互要求。

正常运行时，上层算法只调用：

```text
initRobotHardware()
writeRobotVelocityCommand(cmd)
writeActionCommand(action)
```

FSM、限幅、watchdog 和返回码由适配器检查，不需要人工重复输入模式编号。

## 本轮新增

- `robot_hardware/robot_hardware/robot_test_unitree_h2_velocity_cli.cpp`
  - 通过 `RobotFactory -> RobotHardwareInterface -> UnitreeH2`；
  - 自动要求 FSM 601；
  - 按 20 Hz 调用 `writeRobotVelocityCommand()`；
  - 接受 `vx/vy/omega/duration_ms`；
  - 结束或中断后调用 `StopMove()` 并打印逐次返回值。
- `robot_hardware/robot_hardware/robot_test_unitree_h2_vendor_velocity_cli.cpp`
  - 直接调用官方 `h2::LocoClient::SetVelocity()`；
  - 自动要求 FSM 601；
  - 单次 RPC 的速度和 duration 与官方示例语义一致；
  - 结束后调用 `StopMove()` 并打印返回值。
- `unitreeH2/remote/09_pc2_h2_velocity_probe.sh`
  - 单命令选择 `vendor` 或 `hal`；
  - 自动检查 `MotionSwitcher form=0 name=ai`；
  - 不要求输入长确认短语；
  - 自动保存日志，不写 Stage 06E 成功 gate。
- `unitreeH2/remote/README_R9_SIMPLE_VELOCITY.md`
  - 记录命令、参数和 A/B 结果解释。
- `unitreeH2/docker/repackage_h2_r9_diagnostic_without_docker.ps1`
  - 使用 Windows WSL Ubuntu 22.04 构建，不依赖 Docker Desktop；
  - 基于已验收 r8 包增量重编译并二次解包复验。

## 离线验证结果

- SDK2 快照：`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`
- SDK include-tree SHA256：
  `c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b`
- 两个新增 CLI 在 Ubuntu 22.04/x86_64 编译通过；
- `ldd` 无 `not found`；
- Factory、直接 API 合同和 live motion plan 离线测试通过；
- shell 语法、Python 语法、manifest 和二次解包复验通过。

生成物：

```text
D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r9.tar.gz
SHA256=8fd8fb3d5245a542cf2d0fe6677282bd284a6541b8962781c45e558426407738
```

## 下一步实机 A/B

先在保护支架、脚轮锁定、区域清空、第二操作员持遥控器的条件下，以同一个
`0.20 m/s、1000 ms、x 正方向`参数执行：

```bash
bash scripts/09_pc2_h2_velocity_probe.sh vendor 0.20 0 0 1000
bash scripts/09_pc2_h2_velocity_probe.sh hal    0.20 0 0 1000
```

判定：

- vendor 有动作、HAL 无动作：修复 `UnitreeH2` 适配器、watchdog 或周期调用；
- 两者都有动作：旧 r8/r9 低速验收 profile 是主要问题；
- 两者都无动作且返回 0：继续调查 H2 高层控制消费端、有效速度阈值和并发零速度源，
  不应盲目修改抽象接口；
- `MotionSwitcher` 不是 `ai` 或 FSM 不是 601：测试自动拒绝，不发送非零速度。

当前状态：r9 离线包已完成，实机 A/B 结果待执行并追加。

## 2026-07-22 09:57 首次 vendor 实机结果

执行：

```bash
bash scripts/09_pc2_h2_velocity_probe.sh vendor 0.20 0 0 1000
```

实际输出摘要：

```text
H2_MOTION_SWITCHER_CHECK ret=0 form=0 name=ai
H2_VENDOR_READY fsm_ret=0 fsm_id=0
H2_SIMPLE_PROBE_RESULT backend=vendor rc=65
```

结论：高层 AI motion service 已选中，但机器人当前仍处于 FSM 0（ZeroTorque）。
vendor CLI 在调用 `SetVelocity()` 前正确拒绝，因此本次没有发送非零速度，不能用于
判断 0.20 m/s 是否有效，也不能用于比较 vendor/HAL。下一步应通过原厂遥控流程进入
常规运控 1，并确认 `GetFsmId=601` 后重新执行同一 vendor 命令；本轮不自动从代码
切换 FSM。

远端日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_vendor_20260722_095747_13949.log
```

## 2026-07-22 09:59 同参数 vendor/HAL A/B 结果

测试条件：`vx=0.20 m/s`、`vy=0`、`omega=0`、`duration=1000 ms`，
`MotionSwitcher form=0 name=ai`，当前 FSM 601。

vendor 直接 SDK：

```text
H2_VENDOR_READY fsm_ret=0 fsm_id=601
H2_VENDOR_VELOCITY ret=0 duration_s=1
H2_VENDOR_RESULT velocity_ret=0 stop_ret=0 fsm_after_ret=0 fsm_after=601 interrupted=0
H2_SIMPLE_PROBE_RESULT backend=vendor rc=0
```

HAL 抽象层：

```text
H2_HAL_READY init_ret=0 fsm_ret=0 fsm_id=601
H2_HAL_VELOCITY seq=1..20 ret=0
H2_HAL_STOP action_ret=0 zero_ret=0
H2_HAL_RESULT command_ret=0 stop_ret=0 sent_count=20 expected_count=20 fsm_after_ret=0 fsm_after=601 interrupted=0
H2_SIMPLE_PROBE_RESULT backend=hal rc=0
```

现场观察：两条路径机器人均无物理运动。

远端日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_vendor_20260722_095923_14325.log
/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_hal_20260722_095933_14386.log
```

阶段结论：

- RoboDog HAL 与原厂 SDK 在 0.20 m/s 参数下表现一致；当前证据不支持“HAL 适配器
  吞掉速度”这一假设。
- 原厂 API 返回 0、FSM 前后保持 601，仅证明 RPC 请求被服务接受，不证明步态控制器
  实际采用了速度。
- 下一步用原厂 vendor 路径执行官方文档示例参数 `0.50 m/s、1 s`，只验证是否存在
  有效起步阈值；在该结果出来前不提高 HAL 上限。

## 2026-07-22 10:06 官方示例速度 vendor 结果

测试条件：`vx=0.50 m/s`、`vy=0`、`omega=0`、`duration=1000 ms`，
`MotionSwitcher form=0 name=ai`，当前 FSM 601。本轮直接调用原厂
`unitree::robot::h2::LocoClient::SetVelocity()`，不经过 RoboDog HAL。

```text
H2_MOTION_SWITCHER_CHECK ret=0 form=0 name=ai
H2_VENDOR_READY fsm_ret=0 fsm_id=601
H2_VENDOR_VELOCITY ret=0 duration_s=1
H2_VENDOR_RESULT velocity_ret=0 stop_ret=0 fsm_after_ret=0 fsm_after=601 interrupted=0
H2_SIMPLE_PROBE_RESULT backend=vendor rc=0
```

现场观察：机器人没有物理运动。

远端日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_vendor_20260722_100605_15848.log
```

阶段结论：

- 已执行官方示例量级的 `SetVelocity(0.50, 0, 0, 1.0)`，“0.20 m/s 低于
  有效起步阈值”已不是主要假设。
- `ret=0`、`StopMove ret=0`和 FSM 前后均为 601，证明 SDK 请求/响应调用链
  没有报错；仍不证明 PC1 步态控制器实际采用了速度。
- 当前不应修改 HAL 映射或继续加大速度；下一步是在实机上同时抓取
  `/api/sport/request` 和 `/api/sport/response`，确认 API 7105 请求的实际参数，
  并检查是否存在其他进程持续发送零速度覆盖。

## 2026-07-22 10:19 sport DDS 请求/响应抓取结果

抓取文件：

```text
/home/unitree/p2_unitreeH2/logs/h2_sport_request_20260722_101909.log
/home/unitree/p2_unitreeH2/logs/h2_sport_response_20260722_101909.log
/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_vendor_20260722_101911_19016.log
```

实际非零请求：

```text
api_id: 7105
parameter: '{"duration":1.0,"velocity":[0.5,0.0,0.0]}'
```

对应响应：

```text
api_id: 7105
code: 0
data: ''
```

一秒后由本测试程序显式调用 `StopMove()` 产生的零速度请求：

```text
api_id: 7105
parameter: '{"duration":1.0,"velocity":[0.0,0.0,0.0]}'
```

抓取窗口内未出现第三条 API 7105，因此当前没有证据表明 `dog_cmd`、
遥控器或其他应用通过同一 `sport` API 持续发送零速度覆盖本次命令。
抓取中有多条 API 7001，但它们只是 `GetFsmId`，响应始终为 601，
不是速度覆盖命令。

阶段结论：

- 已从 DDS 话题层确认非零 API 7105 参数正确且服务返回 `code=0`。
- “SDK 没有发出非零参数”、“HAL 吞掉速度”和“同一 sport API 立即零速度
  覆盖”均不受当前证据支持。
- 剩余问题在 PC1 原厂运控服务的命令采用、控制权仲裁、当前运控子模式
  或交付固件与公开 SDK 契约的一致性。

## r9 MotionSwitcher 门禁范围更正

`scripts/09_pc2_h2_velocity_probe.sh` 当前使用完整字符串匹配：

```bash
grep -F 'H2_MOTION_SWITCHER_CHECK ret=0 form=0 name=ai'
```

匹配不成功就在调用 vendor/HAL 执行文件前以 `65` 退出。因此，
**r9 的 `09` 一键测试脚本确实强制 `form=0 name=ai`**。但这不是
`RobotHardwareInterface -> UnitreeH2` 生产适配器的条件，也不是原厂 vendor
速度 CLI 内部条件；后两者的当前非零速度门禁是 FSM 601。

公开 H2 RPC 文档没有把 `form=0 name=ai` 定义为 `SetVelocity()` 的必要
前置条件，所以不应把该字符串硬编码当作通用 H2 控制契约。它在
r9 中只是根据该台交付机当时观测值加入的项目安全门禁。

## MotionSwitcher 返回值与官方契约复核

`ret=0 form=0 name=ai` 不是本项目向机器人赋值的参数。
`robot_test_unitree_h2_motion_mode` 调用只读 `CheckMode(form, name)`，然后打印返回值；
它不调用 `SelectMode()` 或 `ReleaseMode()`。

- `ret` 是 MotionSwitcher RPC 调用结果；`0` 表示查询成功，非零时
  `form/name` 不应被当作可信状态。
- `name=ai` 是该台机器人当时激活的运动服务名称。宇树通用
  MotionSwitcher 示例默认选择 `ai`，但公开 H2 高层 RPC 例程未把
  `name=ai` 列为 `SetVelocity()` 的显式前置条件。
- `form=0` 是服务返回的字符串值；公开 H2 文档未提供其枚举定义，
  不能将其进一步解释为“高层”、“双足”或“普通形态”。

官方当前 H2 `LocoClient` 源码中，`Start()` 只是 `SetFsmId(601)`；
官方 H2 高层示例初始化 `LocoClient` 后直接调用
`SetVelocity()`，没有 MotionSwitcher `form/name` 检查，也没有额外控制权申请。
本机已是 FSM 601，所以“未先调用 `Start()`”不足以解释无动作。

公开 H2 文档和宇树官方 SDK 仓库当前未找到“API 7105 响应
`code=0` 但实机不运动”的 H2 专项故障码或解决步骤。由于请求已达
PC1 `sport` 服务，剩余定位需要比较遥控器可运动时的请求头、优先级/租约、
当前 balance mode 与交付固件版本，必要时向宇树提交实机证据包。

## 2026-07-22 10:39 遥控器 sport 请求抓取

在常规运控状态下对 `/api/sport/request` 进行 15 秒抓取，用户在窗口内
轻推前进摇杆约 1 秒。日志为：

```text
/home/unitree/p2_unitreeH2/logs/h2_remote_sport_request_20260722_103934.log
```

现场确认：轻推遥控器前进摇杆时机器人确实能够行走。抓取文件共 336 行，
所有请求均为 API 7001，`parameter` 为空，且请求头为 `lease.id=0`、
`priority=0`、`noreply=false`；未出现 API 7105。

因此可以确认遥控器速度输入不经过当前抓取的 `/api/sport/request`
API 7105 路径。遥控器链路与 SDK2 `LocoClient::SetVelocity()` 链路是两条
不同的控制输入路径；遥控器能走不能反证 H2 SDK 应改用其他公开速度 API。

SDK2 的 H2 公开头文件仍明确把 `SetVelocity()` 注册并调用为 API 7105，
`Move()` 也只是该函数的封装。目前已证实 API 7105 请求参数正确、服务响应
`code=0`、FSM=601，但实机未采用该速度；故障边界收敛到原厂高层运动服务的
命令采用、控制权仲裁或交付固件与公开 SDK 契约的一致性，而不是 HAL 参数映射。

尝试使用 ROS 2 CLI 读取 `/wirelesscontroller` 时返回：

```text
The passed message type is invalid
```

这表示当前 PC2 的 ROS 2 环境无法为 `unitree_go/msg/WirelessController`
加载有效的 ROS 类型支持，不能据此判断话题没有数据。SDK2 本地包含
`unitree_go::msg::dds_::WirelessController_` 的 DDS 类型，后续应使用 SDK2
原生只读订阅器采集摇杆 `lx/ly/rx/ry/keys`，而不是继续依赖该 ROS 2 CLI。

## 2026-07-22 WirelessController 类型与 ROS CLI daemon 复核

PC2 实际执行结果：

```text
ros2 topic type /wirelesscontroller
  -> xmlrpc.client.Fault: RuntimeError: !rclpy.ok()

ros2 interface show unitree_go/msg/WirelessController
  -> 成功显示 float32 lx/ly/rx/ry、uint16 keys

ros2 pkg prefix unitree_go
  -> /home/unitree/graph_pid_ws/install
```

因此前一轮“消息类型支持缺失”的表述需要收窄：`unitree_go` 包和
`WirelessController` ROS 消息类型实际已经安装成功。失败点是 `ros2 topic type`
访问 ROS 2 CLI daemon 时，daemon 内部 `rclpy` context 已不处于 `ok` 状态；
这不是 H2 DDS 话题或消息定义缺失。下一步先在与审计一致的 Domain 0、
CycloneDDS 配置下重启用户级 `ros2 daemon`，再读取话题；若 CLI 仍异常，
则使用 SDK2 原生只读订阅器绕过 ROS Python/daemon 层。

后续实机复核已完成：在重新导出 Domain 0、CycloneDDS 配置并执行
`ros2 daemon stop/start` 后，CLI 正常返回：

```text
/wirelesscontroller [unitree_go/msg/WirelessController]
unitree_go/msg/WirelessController
```

10 秒只读订阅也成功获得摇杆数据，`ry` 最高约 0.983、`rx` 约
0.404/-0.203、`ly` 约 0.572，并可随摇杆回到 0；`keys` 本次保持 0。
这证明 PC2 的遥控器 DDS/ROS2 状态输入链路正常，前一轮异常仅属于 ROS CLI
daemon context。该结果仍不表示遥控运动是经 API 7105 执行；前述 sport
请求抓取在遥控行走期间仍只有 API 7001。

## 2026-07-22 11:11 官方原版 SDK 实机突破

由固定 SDK2 commit `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` 的
未修改 `h2_loco_client_example.cpp` 编译出的原版基准，在 PC2 通过
`eth0` 查询到 FSM 601，并执行：

```bash
./h2_loco_client_official_21d0a3b2 \
  --network_interface=eth0 \
  --set_velocity="0.5 0 0 1"
```

用户明确确认 H2 实体向前运动。由此 API 7105、SDK2、PC2 到 PC1 的
`sport` RPC 与实体步态执行链已经实机证明可用。

本轮不再把根因归到 PC1 固件拒绝、SDK/API 选择或 FSM 601。当前差异集中在
r9 vendor/HAL 测试包装层，尤其是显式 `StopMove()`、watchdog 和运动启动
时序。用户最后一次成功命令没有紧随 `stop_move`，而前一轮命令序列在
`set_velocity` 返回后立即启动独立 `stop_move` 进程；需要用单变量 A/B
确认究竟是哪一个停止路径过早撤销了速度。

后续 A/B 已确认：相同官方原版非零速度后立即执行独立 `stop_move` 时，
机器人看起来没有运动；只执行非零速度时，机器人明确前进。`tee` 与问题
无关。根因是 `SetVelocity()` 在 RPC 响应后立即返回，CLI 不会等待 JSON
中的 duration 结束，Shell 随即发送的零速度在步态明显启动前撤销了非零命令。

因此此前实机无动作至少包含测试停止时序缺陷。后续测试必须把“命令 duration”
与“客户端观察/兜底停止延迟”分开；生产 HAL 也不能在每次非零速度 API
返回后同步调用 `StopMove()`，停止应由上层显式零速度或独立 watchdog 触发。

随后用户直接运行 r9 `robot_test_unitree_h2_vendor_velocity_cli`，使用同样的
`vx=0.5、duration=1000 ms`，机器人有可见移动，且速度、停止、FSM 复读
全部返回 0/601。这证明 vendor CLI 内部等待 1 秒后 `StopMove()` 的实现可以
产生运动；上一段关于“r9 vendor 可能因一秒停止过早而无动作”的假设被本次
实机结果否定。

当前剩余边界只有两项：先直接执行 HAL 二进制验证抽象接口；再复跑 `09`
包装器判断早先无动作是 MotionSwitcher 前置探测影响，还是当时机器人状态/
人工观察造成的暂态差异。

## HAL 首次复测的参数拒绝

用户首次直接执行 HAL CLI 时使用 `vx=0.5`，程序只输出 Usage，机器人没有
运动。源码复核确认这不是 HAL 运行失败：

- `robot_test_unitree_h2_velocity_cli.cpp` 将测试上限固定为
  `kMaxVx=0.20`；
- `valid()` 在 `abs(vx)>0.20` 时直接返回 false；
- 程序返回 64，未加载 YAML、未创建 `UnitreeH2`、未调用
  `initRobotHardware()`，也未发送任何 API 7105。

因此该轮不能记为 HAL 无动作。下一轮必须使用允许范围内的速度，例如
`vx=0.20、duration_ms=2000`；若仍无动作，再判断 0.20 是否低于该固件的
可见步态阈值，并准备基于官方已验证 0.5 m/s 的新验收上限。

## HAL 0.20 m/s、2 秒实机结果

修正参数后，HAL 测试完成了有效执行：

- `initRobotHardware()` 成功；
- FSM 初始化和结束均为 601；
- `writeRobotVelocityCommand()` 以 20 Hz 调用 40 次，全部返回 0；
- `writeActionCommand(stop_move)` 和最终零速度均返回 0；
- 实体机器人没有响应、没有可见移动。

这证明抽象层调用链确实运行，但不能仅凭返回 0 判定速度被步态控制器采用。
与已经移动的 r9 vendor 测试相比，当前同时存在两个变量：

1. vendor 成功参数为 `vx=0.5、duration=1.0s`，HAL 为 `vx=0.2`；
2. vendor 只发送一次 `SetVelocity`，HAL 每 50 ms 重发一次，且每包
   `duration=0.30s`。

因此下一步先执行单次 vendor `vx=0.20、duration=2.0s`。若 vendor 也不动，
优先归因于该 H2 高层步态的有效速度/起步阈值；若 vendor 能动，则根因是
HAL 的高频重复 RPC 或过短的单包 duration，不是抽象参数本身。

## Vendor 同条件速度阈值 A/B

用户随后在同一个 r9 vendor CLI、FSM 601、`duration=2 s` 下完成单变量
实机 A/B：

```text
vx=0.20 m/s -> 所有 RPC/FSM 返回正常，实体不移动
vx=0.50 m/s -> 所有 RPC/FSM 返回正常，实体明确移动
```

因此当前 HAL `0.20 m/s` 无动作首先由已复现的高层步态有效起步速度解释，
不支持“HAL 高频重发导致不动”的判断。`0.50 m/s` 同时得到宇树未修改官方
样例和 r9 vendor CLI 两条实机证据。

r11 变更边界：

- `UnitreeH2` 代码绝对 vx 上限从 0.20 提高到已验证的 0.50；
- HAL 速度 CLI 验收上限同步提高到 0.50；
- 正式 YAML 默认仍为 0.20，不自动改变部署速度；
- `09` 包装器删除非官方 MotionSwitcher `form=0 name=ai` 强制门禁；
- 更新离线契约：0.50 必须可初始化并正确限幅，0.51 必须拒绝。

直接 API 假 SDK 契约已在 WSL Ubuntu 22.04 重新编译运行并通过：

```text
UNITREE_H2_DIRECT_API_CONTRACT_OK
H2_R11_DIRECT_API_CONTRACT_OFFLINE_OK
```

## r11 抽象 HAL 实机闭环结论

用户把 r11 上传 H2 PC2 后完成归档及包内 manifest 校验，并执行：

```bash
bash scripts/09_pc2_h2_velocity_probe.sh hal 0.50 0 0 1000
```

实际结果：

- `RobotFactory::RobotAllocate()` 创建 H2 对象成功；
- `initRobotHardware()` 返回 0，FSM 为 601；
- `writeRobotVelocityCommand()` 以 20 Hz 写入 20 次，20/20 返回 0；
- `writeActionCommand(stop_move)` 与结束零速度写入均返回 0；
- 结束 FSM 仍为 601，无中断；
- 实体 H2 明确向前移动，并在命令结束后停止。

PC2 日志：

`/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_hal_20260722_113813_37555.log`

最终根因闭环：旧测试使用的 `0.20 m/s` 在当前交付 H2 和当前高层运控下未
触发可见步态；不是 SDK、DDS、FSM 601、RobotFactory、抽象接口映射或 HAL
20 Hz 重发链路未打通。r11 的 `0.50 m/s` 已证明 RoboDog HAL 前向速度最小
闭环可以实机工作。

停止仍应按组合流程描述：本轮连续执行了 `stop_move`、零速度，同时存在 SDK
duration 和 watchdog，尚未隔离证明机器人停止只由其中某一个机制产生。
