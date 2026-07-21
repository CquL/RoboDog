# Unitree H2 Stage 06D r8 实机验收记录

日期：2026-07-21

## 计划

在同一次 H2 PC2 启动、同一个 r8 release 下，先通过 Stage 06C getter-only 门禁，再调用抽象硬件层的零速度命令和 `StopMove()`，确认直接 SDK 控制路径不会造成非预期运动。Stage 06D 不发送非零速度，不验证机器人位移，也不测试状态切换动作。

## 实际执行结果

PC2 当前 release：

```text
/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8
```

终端返回：

```text
H2_GETTER_ONLY_RPC_OK
COMMAND_RC[getter_audit]=0
GETTER_AUDIT_GATE_OK fsm_id=601 fsm_mode=0 mode_value_observation_only=1
Observer: type NO_UNEXPECTED_MOTION_OBSERVED exactly: NO_UNEXPECTED_MOTION_OBSERVED
GATE_WRITTEN=/home/unitree/p2_unitreeH2/build/h2_control_gate_state/stage06d.ok
H2_STAGE06D_ZERO_STOP_OBSERVED_OK fsm_id=601
```

## 结论

Stage 06D 通过。实机保持 FSM 601；零速度与 `StopMove()` 直接 SDK 调用完成，观察期间没有非预期运动，`stage06d.ok` 已生成。该结果允许进入 Stage 06E 的单轴、限幅、限时非零速度流测试，但不代表非零运动已经成功，也不代表所有动作接口已经验收。

## 下一步

在保护架、四个脚轮锁定、测试区域清空、第二操作员持原厂遥控器并掌握停止/阻尼操作的条件下，只测试 `x-positive`。r8 默认参数为 0.08 m/s、20 Hz、1000 ms，共预期 20 次抽象速度接口调用。若没有物理动作，必须记录 `NO_PHYSICAL_MOTION_OBSERVED`，不得生成 Stage 06E 成功门禁；只有确实观察到安全的物理动作后，才能确认 `BOUNDED_STREAM_OBSERVED_SAFE`。
