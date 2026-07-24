# Unitree H2 r11 抽象 HAL 速度实机验证准备记录

日期：2026-07-22

## 本阶段计划

1. 用宇树未修改的 H2 `h2_loco_client` 样例确认原厂 SDK 高层速度接口可驱动实机。
2. 用项目的 vendor CLI 排除测试脚本、DDS 网卡和 RPC 参数问题。
3. 对比 `0.20 m/s` 与 `0.50 m/s` 的实体响应。
4. 提高项目代码的受审计 vx 绝对上限，并重新编译 `librobot_hardware.so`。
5. 生成不覆盖历史 r9 的 r11 PC2 原生运行包，离线验收后再测试抽象 HAL。

## 实际完成

- 宇树未修改官方样例执行 `SetVelocity(0.50, 0, 0, 1)`，H2 实体前进。
- 项目 r9 vendor CLI 执行 `vx=0.50 m/s, duration=2 s`，H2 实体移动。
- 同一个 vendor CLI 执行 `vx=0.20 m/s, duration=2 s`，SDK 返回 0，但实体无可见移动。
- 确认此前“返回 0 但不动”不是 SDK、`eth0`、FSM 601 或参数映射整体未打通，而是 `0.20 m/s` 在当前实机/当前运控版本下未产生可见步态。
- `UnitreeH2` 的 vx 代码绝对上限由 `0.20` 提高到经实机 A/B 验证的 `0.50 m/s`。
- 正式 YAML 部署默认值仍保持 `max_vx: 0.20`，避免升级包自动提高生产速度。
- HAL 测试 CLI 上限提高到 `0.50 m/s`，可专门完成抽象接口实机验收。
- r11 的 `09_pc2_h2_velocity_probe.sh` 删除了非官方前置条件 MotionSwitcher `form=0/name=ai` 强制门禁；仍保留 FSM 601、SDK 返回值和结束停止检查。

## r11 构建与离线验收

产物：

`unitreeH2/runtime_bundle/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r11.tar.gz`

SHA256：

`c90e169afc280786e445cfbc118ec418f8cf4cf64768c772640c7707860564d7`

r11 从已验收 r9 依赖包派生，但重新编译并替换了以下项目产物：

- `lib/librobot_hardware.so`
- `bin/robot_test_unitree_h2_velocity_cli`
- `bin/unitree_h2_factory_contract_test`
- `bin/unitree_h2_direct_api_contract_test`

离线验收结果：

- `UNITREE_H2_FACTORY_CONTRACT_OK`
- `UNITREE_H2_DIRECT_API_CONTRACT_OK`
- 打包前和解包后的 `meta/manifest.sha256` 均通过。
- 新二进制 `ldd` 均无 `not found`。
- CLI 对 `vx=0.51` 返回 64 并拒绝；`vx=0.50` 通过参数边界检查。
- r9 历史归档未被覆盖。

## 当前结论

2026-07-22 11:38，r11 已在 H2 PC2 完成实机验收。已实机证明完整路径：

`测试程序 -> RobotFactory::RobotAllocate -> RobotHardwareInterface -> UnitreeH2::initRobotHardware -> UnitreeH2::writeRobotVelocityCommand -> h2::LocoClient::SetVelocity -> PC1 sport -> H2 移动`

组合停止流程也已执行成功：

`RobotHardwareInterface::writeActionCommand(stop_move) + 零速度写入 -> H2 停止`

因此可以宣称：H2 已接入项目原有工厂和抽象底层运动协议；前向速度接口已在实机打通，组合停止流程也已成功使机器人停止。上层算法可继续使用原有公共 API，不需要直接发布原厂 ROS 2 控制话题。由于停止流程连续执行了 `stop_move`、零速度写入，同时还存在 SDK velocity duration 和 watchdog，本次不能把实体停止单独归因于 `stop_move` 一个调用。

## 实机执行与结果

物理前置条件：常规运控 1、FSM 601、平整空旷场地、第二操作员持原厂遥控器并能立即进入阻尼。

实际执行：

```bash
bash scripts/09_pc2_h2_velocity_probe.sh hal 0.50 0 0 1000
```

实际程序结果：

1. r11 归档 SHA256 校验通过，包内 manifest 全部通过。
2. `RobotFactory::RobotAllocate()` 成功创建 `UnitreeH2`。
3. `initRobotHardware()` 返回 0，初始化前后 FSM 均为 601。
4. `writeRobotVelocityCommand()` 以 20 Hz 执行 20 次，20/20 全部返回 0。
5. `writeActionCommand(stop_move)` 返回 0，结束零速度写入返回 0。
6. `H2_HAL_RESULT`：`command_ret=0 stop_ret=0 sent_count=20 expected_count=20 fsm_after_ret=0 fsm_after=601 interrupted=0`。
7. 实体观察：H2 明确向前移动，并在命令结束后停止。
8. PC2 日志：`/home/unitree/p2_unitreeH2/logs/h2_pc2_velocity_probe_hal_20260722_113813_37555.log`。

## 实机日志本地归档

PC2 日志已通过 SCP 下载到：

`D:\Desktop\RoboDog\unitreeH2\remote\h2_pc2_velocity_probe_hal_20260722_113813_37555.log`

文件大小：`934 bytes`

SHA256：

`3837971954fef77006b2c0e996a0f5355142d2aa7282810358916b93a74a70b5`

本地复核内容与终端输出一致：初始化成功、FSM 601、20/20 次速度写入返回 0、组合停止返回 0、结束 FSM 601、无中断。

## 验收边界与后续计划

本次已经验收 `initRobotHardware()`、前向 `writeRobotVelocityCommand()`，并确认 `writeActionCommand("stop_move")` 返回成功且组合停止流程使实体停止。以下项目仍须分开测试，不能由本次结果自动推导为已完成：

- 横移 `vy` 和转向 `omega` 的实体方向、符号与限幅；
- `stand_up`、`start`、`damp`、`squat`、`sit` 等状态改变动作；
- 单独隔离 `stop_move`、零速度、SDK duration 和 watchdog 对实体停止的作用；
- 长时间连续速度输入、上层算法断流以及 watchdog 停止；
- H2 传感器生产状态源、导航算法和 Docker 完整运行链；
- 无线网络中断后的运行安全和恢复流程。

下一阶段应先做低风险的单轴横移与转向短脉冲测试，再做 watchdog 断流测试。状态改变动作、传感器和 Docker 分别建立新的阶段门禁与日志，不与本次速度接口验收混为一项。
