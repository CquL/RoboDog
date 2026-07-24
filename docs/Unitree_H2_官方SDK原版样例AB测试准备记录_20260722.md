# Unitree H2 官方 SDK 原版样例 A/B 测试准备记录

日期：2026-07-22

## 计划

1. 区分官网“快速开发”底层关节例程与高层 RPC 行走例程；
2. 将官方 `h2_loco_client_example.cpp` 与 r9 vendor/HAL 测试逐行对照；
3. 编译未修改的官方高层样例，形成可传输的 PC2 基准二进制；
4. 在实机上先查询 FSM，再执行官网给出的 `0.5 m/s、1 s` 速度命令；
5. 以物理动作和 DDS 抓取为准，不把示例程序输出当作运动成功证据。

## 官方文档结论

- “快速开发”页运行 `h2_ankle_swing_example`。这是 `rt/lowcmd` 底层
  关节例程，需要机器人吊起、先进入阻尼，再进入调试模式；它不能验证
  RoboDog 的三维高层速度抽象。
- “H2 RPC 例程”使用 `h2_loco_client`，服务名为 `sport`，明确说明无需
  进入调试模式。官网速度命令为：

```bash
./h2_loco_client --network_interface=eth0 --set_velocity="0.5 0 0 1"
```

- 官方源码只执行 `ChannelFactory::Init(0, nic)`、`LocoClient::Init()`、
  `SetTimeout(10.f)`，随后直接调用 `SetVelocity()`。没有 MotionSwitcher、
  `form/name` 或 BalanceMode 前置门禁，也不会自动调用 `Start()`。
- `Start()` 只是 `SetFsmId(601)`；实机已经是 FSM 601 时不是必要前置。

## 与 r9 测试对照

r9 vendor CLI 与官方原版的 DDS 初始化和 `SetVelocity(vx, vy, omega,
duration)` 完全一致。r9 额外增加：

- `GetFsmId()==601` 门禁；
- 2 秒超时，而官方为 10 秒；
- 等待 duration 后显式 `StopMove()`；
- 外层 `09` 脚本的 MotionSwitcher 只读检查和固定字符串门禁。

这些差异没有发现能够解释“API 7105 返回 0 但实机不动”的协议错误。
因此运行官方原版的价值是最终排除测试包装层，并形成可提交宇树支持的
最小复现；预期仍可能复现无动作。

## 实际完成

已从本地固定的官方 SDK2 commit
`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` 未修改源码编译：

`unitreeH2/official_sample_baseline/r10/h2_loco_client_official_21d0a3b2`

哈希：

- 源文件：`a91791a2511e9c7927d321aaeb2c404c81a5e306701e6cc1d546e62421b3a0f5`
- SDK 静态库：`08402aea74150dfbfc3fbfded4ca746916a8d892b54d2bade0cbf392a3be4029`
- 二进制：`0de8dce74bbf64d95816d0beb5d19ded01f1991f6041449a7e141ced206759c6`

离线 `ldd` 已使用 r9 release 的 DDS 动态库验证，无 `not found`。

## 当前状态

官方原版基准已准备并完成离线链接验证，尚未在 H2 PC2 执行，因此尚无
实机动作结论。下一步只上传该基准二进制和 SHA256 文件，不安装 SDK、
不执行 `sudo make install`、不覆盖 PC2 原厂 `/opt/unitree_robotics`。

## 2026-07-22 11:11 实机结果

官方原版基准已上传到：

```text
/home/unitree/p2_unitreeH2/build/official_sdk_baseline_r10
```

实机验证结果：

- SHA256 严格校验通过；
- `ldd` 的 `libddscxx.so.0`、`libddsc.so.0` 均解析到 r9 release，
  没有缺失依赖；
- 官方原版 `--get_fsm_id` 返回 601；
- 官方原版执行 `--set_velocity="0.5 0 0 1"` 后，用户明确观察到
  **H2 向前运动**。

这首次形成了 H2 高层非零速度的实体动作证据。因此下列链路已经实机打通：

```text
PC2 eth0
  -> 官方 SDK2 h2::LocoClient
  -> sport RPC / API 7105
  -> PC1 高层运动控制
  -> H2 实体向前运动
```

该结果排除了“公开 API 7105 不适用于交付 H2”“FSM 601 不能使用 SDK
速度控制”“PC1 固件完全拒绝外部 SDK 速度”等假设。

现场命令序列还揭示了一个需要继续隔离的时序差异：用户先执行了一轮
`set_velocity` 后立即单独执行 `stop_move`，随后又执行一轮只有
`set_velocity`、没有紧随其后的 `stop_move`；用户在后一阶段报告前进。
因此当前首要怀疑是测试包装层的停止时序，而不是速度 API 映射：

- 官方原版返回后，速度命令由 PC1 继续采用；
- r9 vendor CLI 等待 `duration_ms` 后主动调用 `StopMove()`；
- r9 HAL 测试结束时也主动发送停止，且有 watchdog；
- 如果 H2 步态从接收速度到实际迈步存在启动延迟，测试程序可能在第一步
  明显发生前就发送了停止。

下一步需在同一常规运控状态下，先直接运行 r9 vendor 二进制（绕过
MotionSwitcher 包装脚本），再制作只改变停止时序的最小 A/B。不能再把
问题归因于宇树 SDK 或 API 7105 本身。

## 2026-07-22 11:13 立即 StopMove 的因果确认

用户随后明确复现了以下 A/B：

1. 在同一个 Shell 命令块中依次执行 `set_velocity` 和 `stop_move`，机器人
   看起来没有运动；
2. 只执行相同的 `set_velocity="0.5 0 0 1"`，机器人向前运动。

原因不是 `tee`。官方 `SetVelocity()` 在收到 RPC 响应后即返回；JSON 中的
`duration=1` 交给 PC1 运动服务处理，不会使 CLI 在本地阻塞等待 1 秒。
因此第一组命令实际形成：

```text
发送非零 API 7105
  -> 收到 RPC 响应，CLI 立即退出
  -> Shell 立即启动下一进程
  -> 发送 StopMove 的零速度 API 7105
  -> 非零速度在明显迈步前被撤销
```

修正后的安全测试必须在停止前留出独立观察窗口，例如对 1 秒速度命令等待
2 秒后再补发兜底停止：

```bash
./h2_loco_client_official_21d0a3b2 \
  --network_interface=eth0 --set_velocity="0.5 0 0 1"
sleep 2
./h2_loco_client_official_21d0a3b2 \
  --network_interface=eth0 --stop_move
```

这也解释了官方原版“单独发送能走”而紧邻停止命令“看起来不动”的差异。
r9 vendor CLI 虽然不是立即停止，但会在 `duration_ms` 到期的同一时刻主动
`StopMove()`；r9 HAL 也在短测试窗口结束后立即停止。二者需要增加明确的
启动/观察余量，不能把过早停止后的无动作判为 HAL 或 SDK 失败。

## 2026-07-22 r9 vendor 直接调用复核

用户绕过 `09_pc2_h2_velocity_probe.sh`，直接执行 r9 vendor 二进制：

```text
robot_test_unitree_h2_vendor_velocity_cli
  --interface eth0 --vx 0.5 --vy 0 --omega 0
  --duration-ms 1000 --execute
```

返回 `fsm_id=601`、`velocity_ret=0`、`stop_ret=0`，用户明确确认
**机器人有移动**。

因此需要修正上一节的范围：紧邻的独立 `stop_move` 确实会取消官方原版
命令，但 r9 vendor CLI 内部等待 1 秒后再停止的时序能够产生可见运动，
不是 vendor 无动作的根因。当前已经实机通过两条高层路径：

```text
官方未修改 h2_loco_client -> 实体移动
r9 vendor CLI              -> 实体移动
```

下一步进入真正的 HAL A/B，不再修改 SDK API 映射：直接绕过 `09` 包装脚本，
执行 `robot_test_unitree_h2_velocity_cli`，验证 `RobotFactory -> UnitreeH2 ->
initRobotHardware -> writeRobotVelocityCommand`。
