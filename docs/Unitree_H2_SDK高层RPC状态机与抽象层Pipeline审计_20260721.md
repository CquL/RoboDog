# Unitree H2 SDK 高层 RPC、状态机与抽象层 Pipeline 审计

日期：2026-07-21  
范围：H2 高层运动 SDK、`robot_hardware` 三项抽象接口、r8 实机结果  
当前结论：接口调用链已经打通到原厂 `sport` RPC，但实机非零运动尚未验收通过。

## 1. 本轮计划与实际工作

计划：

1. 核对宇树 H2 官方开发文档和固定版本 SDK2 源码；
2. 区分高层 `LocoClient` 和低层关节控制的启动条件；
3. 审计三项抽象接口实际匹配的 H2 API、状态门禁和返回值；
4. 解释 FSM 601、703、API ID 和错误码，避免混用；
5. 根据 r8 实机日志判断当前完成度，设计下一步排障门禁。

实际完成：

- 核对了 2026-07-21 的宇树 H2 RPC、快速开发、遥控器和调试规范页面；
- 核对了 SDK2 固定提交 `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` 的 H2 client、API 和错误码头文件；
- 审计了 `UnitreeH2`、工厂、默认 YAML、watchdog、离线合同和实机 tester；
- 复核了 r8 实机附件，SHA256 为
  `99e157c7d8a78a62a3fa8a57944259110e12bc3f702c759e60102c211f731442`；
- 本轮没有修改 H2 运动实现，只新增/补充审计记录。

## 2. 必须先区分的两条开发路径

### 2.1 本项目使用的高层路径

```text
上层算法 vx/vy/omega
  -> RobotHardwareInterface
  -> UnitreeH2
  -> unitree_sdk2::h2::LocoClient
  -> CycloneDDS RPC，service=sport
  -> PC1 原厂高层运动服务
  -> 机器人运动控制器
```

宇树 H2 RPC 例程明确说明该高层例程“无需进入调试模式”。因此本项目测试
`LocoClient::SetVelocity()` 时，不应为了排障而按 `L2+R2` 释放高层运控。

这条路径不发送 ROS 2 控制 topic，上层算法也不需要依赖 ROS 2 消息控制机器人；但
SDK2 内部仍使用 CycloneDDS RPC，并依赖机器人 PC1 上运行的原厂 `sport` 服务。因此它
是“与 ROS 2 控制话题解耦”，不是“与宇树原厂运行栈完全无关”。

### 2.2 低层关节路径

```text
自定义低层控制器
  -> rt/lowcmd
  -> 电机/关节
  <- rt/lowstate、rt/secondary_imu
```

H2 快速开发中的 `h2_ankle_swing_example` 属于这条路径。官方要求阻尼/零力矩、保护
吊架并进入调试模式，是为了停止开机后原厂运控程序持续发送的零速度，避免两个低层
命令源冲突。本项目当前 H2 HAL 不是这条路径。

## 3. 对附件中旧说明的纠正

| 旧说法 | 正确结论 |
|---|---|
| `Start()` 进入 FSM 4 | 错。官方头文件中 `Start()` 调用 `SetFsmId(601)`；`StandUp()` 才请求 FSM 4。 |
| H2 RPC 没有明确返回值 | 错。getter/setter 返回 `int32_t`；`SetVelocity()` 直接返回底层 `Call()` 的返回码。 |
| 高层 SDK 必须先用遥控器进入常规运控 | 官方没有这一要求；高层模式可由代码请求，但必须检查返回码并复查实际 FSM。 |
| 高层 `LocoClient` 必须进入调试模式 | 错。官方明确高层 RPC 例程无需进入调试模式。 |
| 601 是官方规定的所有速度命令唯一前置状态 | 证据不足。官方只明确 `Start()->601`；本项目把 601 设为安全门禁。 |
| `fsm_mode=0` 表示某个确定工作模式 | 官方公开资料没有给出该数值语义，目前只能记录，不能作为控制判据。 |

## 4. 不要混淆三类数字

### 4.1 FSM ID：机器人运行状态

官方 H2 `LocoClient` 静态封装可以确认：

| 代码调用 | 请求的 FSM ID | 说明 |
|---|---:|---|
| `ZeroTorque()` | 0 | SDK 头文件的静态映射 |
| `Damp()` | 1 | 阻尼/Passive 类状态 |
| `Squat()` | 2 | SDK 头文件的静态映射 |
| `Sit()` | 3 | 坐下 |
| `StandUp()` | 4 | 站立 |
| `Start()` | 601 | 启动高层运动 FSM |

r8 实机 `GetAvailableFsmIds()` 返回的交付固件名称包括：

```text
0 Invalid
1 Passive
2 Protection
3 Sit
4 FixStand
5 HybridPassive
100 BeyondMimic
502 HumanMimic
503 HumanMimic2
601 HybridWalk
701 WalkNew
703 PhaseWalk
```

这里出现了固件运行时名称与 SDK 便捷函数命名并不完全一致的情况，例如 SDK 的
`ZeroTorque()->0`，实机却把 0 报告为 `Invalid`；SDK 的 `Squat()->2`，实机把 2 报告
为 `Protection`。所以状态语义必须以当前固件的 `GetAvailableFsmIds()`、动作返回码和
实机验证为准，不能仅靠一张固定表推断。

601 和 703 的当前证据边界：

- SDK 公开代码只证明 `Start()` 请求 601；
- 本机遥控“常规运控1”曾观测到 601 `HybridWalk`；
- 本机遥控“常规运控2”曾观测到 703 `PhaseWalk`；
- 703 没有在公开 H2 client 头文件中定义，不能推断它是 `SetVelocity` 的前置条件；
- 当前 `required_motion_fsm_id=601` 是本项目保守安全门禁，不是 SDK 客户端硬编码要求。

### 4.2 API ID：RPC 方法编号

这些不是机器人状态：

| API | API ID |
|---|---:|
| `GetFsmId` | 7001 |
| `GetFsmMode` | 7002 |
| `GetBalanceMode` | 7003 |
| `GetAvailableFsmIds` | 7008 |
| `SetFsmId` | 7101 |
| `SetVelocity` | 7105 |

### 4.3 返回码：一次 RPC 调用的结果

- `0`：SDK/RPC 调用成功；不等于机器人已经产生物理运动；
- H2 loco：`7301` 状态不可用，`7302` FSM ID 无效，`7303` task ID 无效；
- 常见 SDK client：`3102` 发送失败，`3103` API 未注册，`3104` 调用超时，
  `3105` 响应 API 不匹配，`3106` 响应数据错误，`3107` lease 无效；
- 常见 server：`3201` 发送响应失败，`3202` 服务内部错误，`3203` API 未实现，
  `3204` 参数错误，`3205` lease 被拒绝，`3206` lease 不存在，`3207` lease 已存在。

离线合同输出中的 `-55/-56/-66/-77/-88/-99` 是 fake SDK 人工注入的测试错误，
不是宇树官方错误码表。

## 5. 三项抽象接口当前如何匹配 H2

### 5.1 `initRobotHardware()`

当前调用链：

```text
检查 YAML
  -> ChannelFactory::Init(domain_id, network_interface)
  -> 构造 LocoClient
  -> LocoClient::Init()
  -> SetTimeout()
  -> 可选 GetFsmId()
  -> 启动本地 watchdog
```

注意：`LocoClient::Init()` 是 `void`，作用是注册 RPC API；它不会让机器人站立，也不会
启动步态。当前 `initRobotHardware()` 也不会自动调用 `StandUp()` 或 `Start()`。

项目返回：配置/初始化异常为 `1001`；FSM 查询失败，或运动授权打开但 FSM 不是 601，
为 `1010`；成功为 `0`。

### 5.2 `writeRobotVelocityCommand()`

当前调用链：

```text
检查有限数值
  -> 按工程上限钳制 vx/vy/omega
  -> 非零命令检查 allow_motion_commands
  -> 每条非零命令 GetFsmId()
  -> 要求 FSM == required_motion_fsm_id（当前 601）
  -> LocoClient::SetVelocity(vx, vy, omega, duration)
  -> 本地 watchdog 负责陈旧命令 StopMove
```

项目返回：未授权 `1008`；FSM 不就绪 `1010`；非法值、厂商非零返回或异常 `1005`；
厂商返回 0 才转换为项目成功 0。

### 5.3 `writeActionCommand()`

| 抽象动作字符串 | H2 API |
|---|---|
| `stop_move` | `StopMove()`，实质是零 `SetVelocity` |
| `stand_up` | 先 `StopMove()`，再 `StandUp()`，请求 FSM 4 |
| `prepare_motion` | 先 `StopMove()`，再 `Start()`，请求 FSM 601 |
| `damp` | 先 `StopMove()`，再 `Damp()`，请求 FSM 1 |
| `squat` | 先 `StopMove()`，再 `Squat()`，请求 FSM 2 |
| `sit` | 先 `StopMove()`，再 `Sit()`，请求 FSM 3 |

项目返回：状态动作未授权 `1008`；不支持 `1007`；停止失败 `1006`；动作失败 `1004`。

## 6. 抽象设计是否实现机器人解耦

名义控制层已经实现了解耦：`RobotFactory` 仍保留原来的头文件内可见分配逻辑，
`robot_model: unitree_h2` 创建 `UnitreeH2`。导航算法只产生统一的
`RobotVelocityCommand` 和动作字符串，不需要直接包含宇树 SDK 头，也不需要输出原厂
ROS 2 控制 topic。以后增加机器人仍可独立继承 `RobotHardwareInterface`。

但当前还不能称为“完整的代码自主启动生命周期”，原因是：

1. 默认 YAML 的 `allow_motion_commands` 和 `allow_state_changing_actions` 都是 `false`；
2. `initRobotHardware()` 不切换状态；
3. 打开运动授权后，初始化反而要求机器人已经是 601；
4. `prepare_motion/Start()` 返回 0 后，当前代码没有轮询确认 FSM 已经收敛到 601；
5. 抽象层没有实际速度、足端接触、故障和控制权竞争反馈，无法用返回 0 证明机器人动了；
6. 默认 SDK timeout 为 10 秒，而 watchdog 为 250 ms；RPC 若阻塞，二者共用 mutex，
   250 ms watchdog 不能保证按时进入停止调用。

因此三项纯虚接口可继续保留，不需要破坏其他机器人；H2 内部还需要补一段显式、可验收
的状态准备流程。不要在 `initRobotHardware()` 中默认静默改变机器人姿态。建议把
`prepare_motion` 定义为显式动作：根据当前 FSM 请求经过评审的状态切换，轮询确认目标
FSM，然后才打开速度门。未来如需通用状态反馈，可给基类增加有默认实现的能力/健康
查询接口，避免新增纯虚函数导致所有旧机器人同时修改。

## 7. 推荐的生产控制 Pipeline

```text
1. 物理保护和原厂遥控器急停人员就位
2. PC2/容器绑定 H2 通信网卡 eth0
3. ChannelFactory 初始化一次
4. LocoClient Init + 合理的短 RPC timeout
5. GetAvailableFsmIds/GetFsmId/GetFsmMode/GetBalanceMode
6. 显式 prepare_motion
   - 已是 601：不切换
   - 其他状态：只执行经过该固件实机验证的转换；每一步检查 ret 并轮询 FSM
   - 未知/703/故障状态：停止，不盲目 SetFsmId
7. 只有确认目标运动状态后，允许算法周期发送 vx/vy/omega
8. 每条命令检查返回码；命令陈旧时 watchdog 发送 StopMove
9. 正常停止发送零速度 + StopMove，并读取状态/运动反馈确认
10. 阻尼属于会改变承重状态的显式人工批准动作，不能在析构时自动调用
```

代码可以调用 `StandUp()`、`Start()` 等高层模式接口，所以遥控器不是正常算法命令的
必需来源；但安全上仍必须保留原厂遥控器/硬件停止手段。宇树公开资料没有规定所有固件
都必须执行固定的 `StandUp()->Start()` 顺序，本机具体转换必须逐状态小步验证或向宇树
确认，不能凭猜测自动化。

## 8. r8 实机结果

测试参数：

```text
axis=x-positive
vx=0.080 m/s, vy=0, omega=0
stream=1000 ms
frequency=20 Hz
vendor duration=0.300 s
watchdog=150 ms
SDK timeout=0.200 s
expected FSM=601
```

已证明：

- 20/20 次抽象速度接口调用均返回 0；
- 最大实测发送间隔 52 ms，小于 100 ms 门限；
- 最大 RPC 延迟 1 ms；
- stream 前后 FSM 都是 601；
- stream 完成和作用域退出时 `StopMove`、零速度均返回 0；
- 观察结果为 `NO_PHYSICAL_MOTION_OBSERVED`；
- 脚本正确输出 `stage06e_gate_written=0`，没有生成 `stage06e.ok`。

这次只证明：

```text
RobotHardwareInterface -> UnitreeH2 -> SetVelocity -> sport RPC 返回 0
```

没有证明：

```text
PC1 运动控制器采纳命令 -> 产生踏步 -> 机器人产生实际速度/位移
```

因此当前状态是“SDK/RPC 通路通过，物理运动验收失败”，不能写成“H2 已可运动控制”。

## 9. 下一步排障计划

在再次增加速度前，先做同一物理保护条件下的诊断性 A/B：

1. 新建受保护的状态动作门禁，仅测试一次 `Start()`，记录调用前后
   `GetFsmId/GetFsmMode/GetBalanceMode`、返回码和状态收敛时间；
2. 使用宇树原始 `h2_loco_client_example` 与本项目 HAL，在同一 FSM、同一网卡、同一极小
   参数下各运行一次；若两者都无动作，优先排查固件/控制权/有效速度阈值，不先改 HAL；
3. 检查是否存在原厂或遥控程序持续覆盖零速度，并要求宇树确认该交付固件的高层命令
   仲裁、有效速度死区、保护架/足端接触约束；
4. 补充机器人实际速度或位移反馈。只有 RPC 成功并且观测反馈一致，才能生成 06E gate；
5. 未完成上述诊断前，不把 `0.08` 直接提高为更大脉冲，也不盲目切换到 703。

## 10. 依据

- 宇树 H2 RPC 例程：`https://support.unitree.com/home/zh/H2_developer/rpc_routine`
- 宇树 H2 快速开发：`https://support.unitree.com/home/zh/H2_developer/quick_development`
- 宇树 H2 遥控器：`https://support.unitree.com/home/zh/H2_developer/remote_control`
- 宇树 H2 调试规范：`https://support.unitree.com/home/zh/H2_developer/debugging_specification`
- SDK2 H2 client：固定提交的
  `include/unitree/robot/h2/loco/h2_loco_client.hpp`
- SDK2 H2 API：固定提交的
  `include/unitree/robot/h2/loco/h2_loco_api.hpp`
- 本项目：`include/robot_hardware_interface.h`、`include/robot_factory.h`、
  `src/unitree/unitree_h2.cpp`、`config/unitree_h2.yaml`、
  `robot_test_unitree_h2_live_motion.cpp`。

