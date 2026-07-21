# Unitree H2 PC2 远端文件约定

从 2026-07-16 起，PC2 上本项目的所有新增文件统一放在：

```text
/home/unitree/p2_unitreeH2/
```

目录合同：

```text
/home/unitree/p2_unitreeH2/
├── scripts/   审计、构建和受控测试脚本
├── logs/      原始执行日志，只追加不覆盖
├── src/       后续上传的最小探针源码
├── build/     PC2 本机构建输出，可重建
└── config/    H2 专用运行配置
```

不要把文件继续散落到 `/home/unitree/`。也不要移动或修改原厂的 `graph_pid_ws/`、
`slam_config/`、`p2/`、Anaconda、NoMachine、隐藏配置或 systemd 文件。

Windows 本地的 `unitreeH2/remote/` 保留脚本和回传日志的证据副本，不因 PC2 路径整理
而移动历史文件。已有 PC2 根目录文件使用
`pc2_workspace_migrate_to_p2_unitreeH2.sh` 逐个显式迁移；脚本不使用通配符，也不触碰
清单外文件。

阶段 04 已通过。PC2 缺少 HG `SportModeState_.hpp` 和 H2 Loco 头文件，所以继续
禁止把本地 H2 头文件与 PC2 旧 SDK archive 混合编译。阶段 05 已完成 LowState、
secondary IMU、BMS、MainBoard 的 7-topic 纯读取探针。

经用户单独提出底层控制实机测试后，Stage 06C 至 06E 只允许使用从固定候选镜像导出、
带 manifest/私有动态库/ldd 门禁的自包含 native bundle，解包到 `build/` 下。06C
getter RPC、06D 零速度/StopMove 和 06E 单轴非零脉冲必须分别执行；这不改变“禁止
PC2 旧 SDK 混编”的结论，也不代表 Stage 06B 生产状态源已经完成。

2026-07-20 的 r6 实机链已经完成 06C getter、06D 零速度/`StopMove()` 和一次
`x-positive` 非零 `SetVelocity()` RPC；该次脉冲因 `0.05 m/s + 120 ms` 很小，物理
方向仍未标定。r7 只在本机使用固定 SDK2 头和 r6 私有库重编两个测试 CLI，并提供
受限运行时参数；PC2 仍只接收完整 native bundle，不使用 PC2 旧 SDK 重新编译。

r8 将 06E 从一次极短 RPC 改为固定 `20 Hz` 的有界 stream：`--stream-ms` 默认
`1000`，允许 `250..1000 ms` 且只能按 `50 ms` 步进。每次单条命令的厂商 duration
仍为 `0.300 s`，watchdog 固定 `150 ms`；实际 RPC 数量必须等于
`stream_ms / 50`，实测最大发送间隔不得超过 `100 ms`。这仍是一个轴的一次受控
测试，不是无限连续运动，也不改变 06C/06D 和现场物理门禁。

Stage 06E 的 stream RPC 成功与物理动作观察是两个门槛。若现场没有看到动作，应在方向
提示输入 `NO_PHYSICAL_MOTION_OBSERVED` 或 `NO_VISIBLE_MOTION`；脚本保留 TSV 观察
记录后输出 `H2_STAGE06E_NO_MOTION_OBSERVED` 并非零退出，不写 `stage06e.ok`。只有
明确观察到物理方向并输入 `BOUNDED_STREAM_OBSERVED_SAFE`，才生成 06E 成功门禁。

一次性整理顺序：

```bash
mkdir -p /home/unitree/p2_unitreeH2/{scripts,logs,src,build,config}
bash /home/unitree/p2_unitreeH2/scripts/pc2_workspace_migrate_to_p2_unitreeH2.sh \
  2>&1 | tee /home/unitree/p2_unitreeH2/logs/h2_pc2_workspace_migration_20260716.log
```

阶段 04 ABI 审计（已完成）：

```bash
bash /home/unitree/p2_unitreeH2/scripts/04_pc2_sdk2_abi_read_only_audit.sh \
  2>&1 | tee /home/unitree/p2_unitreeH2/logs/h2_pc2_sdk2_abi_audit_20260716.log
```

阶段 05 远端路径：

```text
scripts/05_pc2_build_run_hg_state_read_only.sh
src/hg_state_probe/CMakeLists.txt
src/hg_state_probe/h2_hg_state_read_only_probe.cpp
```

Runner 会在初始化 DDS 前检查所有源码、PC2 SDK/CMake/DDS 和 CycloneDDS 配置哈希；
随后在 PC2 新目录中原生编译，并检查 `ldd`、禁止发送符号、7-topic 白名单、timeout 和
残留进程。应用没有 DataWriter、发送通道或控制 Client；SDK2 Init 内部可能预建 DDS
Publisher 容器，因此准确边界是“无应用发送路径”，不是“完全不创建 Publisher 实体”。

```bash
set -o pipefail
log=/home/unitree/p2_unitreeH2/logs/h2_pc2_hg_state_probe_20260716.log
bash /home/unitree/p2_unitreeH2/scripts/05_pc2_build_run_hg_state_read_only.sh \
  2>&1 | tee "${log}"
stage05_rc=${PIPESTATUS[0]}
printf 'STAGE05_RUN_RC=%s\n' "${stage05_rc}" | tee -a "${log}"
```

阶段 05 不调用 H2 Loco、`GetFsmId()`、速度、动作、FSM、`rt/lowcmd` 或 `/arm_sdk`，
也不停止任何原厂进程。若失败，不要绕过哈希或二进制门槛；先回传完整日志。
