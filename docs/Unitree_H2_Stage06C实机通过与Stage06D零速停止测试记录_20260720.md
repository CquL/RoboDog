# Unitree H2：Stage 06C 实机通过与 Stage 06D 零速停止测试记录

日期：2026-07-20（Asia/Shanghai）  
状态：Stage 06C、06D、06E 已完成；r6 非零 RPC 成功但物理方向未确认；r7 可配置
06E 已完成本地离线验收，尚未部署到 PC2。

## 1. 目标与边界

本轮验证 `RobotHardwareInterface` 的 Unitree H2 高层运动适配器。应用层只调用
`initRobotHardware()`、`writeRobotVelocityCommand()`、`writeActionCommand()`；H2
适配器在内部使用 Unitree SDK2 `h2::LocoClient` 调用原厂 `sport` 高层 RPC。

本文件不把 SDK2 内部的 CycloneDDS RPC 描述为“没有 DDS”，也不把离线测试描述为实机
运动成功。

安全边界：不发布 `rt/lowcmd`，不进入 `L2+R2` 低层调试模式，不停止原厂服务；遥控器、
硬件急停和防跌倒保护始终保留。`StopMove()` 不是硬件急停。

## 2. 已完成：Stage 06C（只读 getter RPC）

### 2.1 现场启动基线

- 遥控器采用“常规运控 1”；不采用 `L2+R2` 调试模式。
- 实机 FSM：`601 = HybridWalk`。
- `fsm_mode=0` 仅作观测记录，尚未赋予控制语义。
- 当前可用 FSM 列表中同时包含 `601 HybridWalk`、`701 WalkNew`、`703 PhaseWalk` 等共 264 项。

### 2.2 实际执行与结果

PC2 命令：

```bash
bash /home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4/scripts/06_pc2_h2_getters_rpc_gate.sh
```

实机日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_20260720_150601_2977.log
```

关键成功标记：

```text
H2_GETTER_ONLY_RPC_OK
GETTER_AUDIT_GATE_OK fsm_id=601 fsm_mode=0
H2_STAGE06C_GETTER_RPC_OK fsm_id=601 fsm_mode=0
```

结论：`initRobotHardware()` 的实机初始化、SDK2 `LocoClient` 初始化、`sport` RPC 回应以及
FSM 读取均已打通。该阶段没有调用速度、停止、动作或低层关节控制命令。

## 3. Stage 06D 计划：零速度与 StopMove 写入验证

### 3.1 要验证的接口

Stage 06D 首次验证 `writeRobotVelocityCommand()` 的零速度路径，以及适配器的
`StopMove()` 停止路径。该阶段不发送非零 `vx`、`vy` 或 `omega`，也不调用
`StandUp`、`Start`、`Damp`、`Squat`、`Sit`。

这是一条真实写入 RPC，不能视为只读测试；预期不产生位移，但必须由现场人员观察。

### 3.2 执行前必须满足

1. H2 处于常规运控 1，且 06C 的 `fsm_id=601` gate 文件仍有效。
2. 已安装官方防跌倒/保护支架，四个脚轮均已锁定。
3. 周围净空，无人在机器人可达范围内。
4. 第二名操作员全程手持原厂遥控器，已确认阻尼/高层停止流程。
5. 不进入 `L2+R2` 调试模式；不启动 Docker、导航算法或其他控制进程。

### 3.3 待执行命令

在 PC2 的交互式终端执行：

```bash
bash /home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4/scripts/07_pc2_h2_zero_stop_gate.sh
```

脚本会先要求精确输入：

```text
ZERO_STOP_PHYSICAL_GATES_CONFIRMED
```

它在写入后会要求现场观察员精确输入：

```text
NO_UNEXPECTED_MOTION_OBSERVED
```

只有实际未观察到异常运动时才允许输入第二个字符串。成功标记预计为：

```text
H2_STAGE06D_ZERO_STOP_OBSERVED_OK fsm_id=601
```

### 3.4 第一次执行结果：未写入，因启动会话变化安全停止

执行时间：`2026-07-20T16:09:46+08:00`。  
PC2 日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06d_20260720_160946_4853.log
```

脚本在公共前置检查完成后停止，实际输出为：

```text
H2_GATE_FAILED=STAGE-06c-REQUIRED-FIELD-COUNT-0=boot_id=7e1eef1c-bf6d-43dd-bc70-f3444055986c
```

原因：06C 成功时的 PC2 启动会话与本次 06D 的 `BOOT_ID` 不同。06D 设计为只接受同一
启动会话的 `stage06c.ok`，因此拒绝复用旧 gate。这是预期的 fail-closed 行为。

影响与实际边界：脚本在物理确认提示和 `--zero-stop` 子程序之前退出；**没有调用
`writeRobotVelocityCommand()`、没有发送零速度、没有调用 `StopMove()`，也不需要进行
“未观察到异常运动”确认。**

恢复步骤：保持常规运控 1，先在当前 PC2 启动会话重新完成 Stage 06C，再重新执行
Stage 06D；不要手工复制、编辑或伪造 gate 文件。

### 3.5 当前启动会话的 06C：通过；06D：TTY 门禁停止，未写入

当前 PC2 启动会话 `BOOT_ID`：

```text
7e1eef1c-bf6d-43dd-bc70-f3444055986c
```

重新执行的 06C 日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_20260720_161826_6971.log
```

结果：06C 通过，写入了同一启动会话的 `stage06c.ok`，其 `fsm_id=601`、
`getter_only_rpc_ok=1`。

紧接着执行的 06D 日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06d_20260720_161832_7118.log
```

06D 在离线合同检查完成后，以以下标记停止：

```text
H2_GATE_FAILED=STAGE06D_REQUIRES_INTERACTIVE_TTY
```

原因：06D 必须在 stdin 和 stdout 都是交互式 TTY 的终端运行，才能收集两次人工安全确认。
本次会话未满足该条件。停止点在物理确认提示之前；**没有执行 `--zero-stop`、没有发送零
速度、没有调用 `StopMove()`，也没有机器人运动。**

下一步只允许先诊断当前终端是否分配伪终端（PTY）；不要用 `script`、管道、重定向或任何
绕过方式伪造 TTY。确认真实交互终端后，才重新执行 06D。

### 3.6 06D TTY 门禁缺陷：根因、修复与 r5 打包状态

现场 TTY 诊断已证明 Xshell 会话本身正常：

```text
/dev/pts/1
STDIN_TTY_OK
STDOUT_TTY_OK
```

根因位于 r4 的共享脚本：`h2_prepare_gate()` 先执行 `exec > >(tee -a "$H2_LOG") 2>&1`，
把 stdout 从终端改成管道；06D 随后错误地检查 `-t 1`，因此必然误报
`STAGE06D_REQUIRES_INTERACTIVE_TTY`。

本地修复：

- `h2_prepare_gate()` 在 `tee` 重定向前保存原始 stdout 到 FD 3：`exec 3>&1`；
- 06D 改为检查 `-t 0 && -t 3`；
- 门禁离线回归新增对上述两行的检查。

本地无 Docker 验证结果：

```text
BASH_SYNTAX_OK
H2_GATE_SCHEMA_OFFLINE_OK
```

计划中的新包名为：

```text
unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5
```

截至本次记录，r5 **尚未生成**。阻塞原因是 Windows 本机 Docker Desktop Linux Engine 未运行：

```text
failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine
Image not found: unitree_h2:amd64-live-test-candidate
```

不允许手工修改 r4 目录或手工打 tar 绕过 manifest、候选镜像和离线合同验证。待 Docker
Engine 恢复后，必须重新构建并离线验收 r5，再上传 PC2；在此之前不得重新运行 06D。

### 3.7 r5 无 Docker 重新封装：已完成并可部署

为避免把 PC2 原生运控测试错误依赖于 Windows Docker，r5 改为从已验收的 r4 归档进行
可追溯重新封装，而不是重编译或替换控制二进制。

输入父包：

```text
unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz
sha256=ddbff7ae8012c745daa175517f2e16ceb9605c97045cf60ac2068d43a5639b60
```

r5 仅替换以下内容：

1. `scripts/h2_pc2_hal_gate_common.sh`：在日志 `tee` 重定向前保存原始 stdout 到 FD 3；
2. `scripts/07_pc2_h2_zero_stop_gate.sh`：TTY 检查由 `-t 1` 改为 `-t 3`；
3. `scripts/tests/test_h2_gate_schema_offline.sh`：新增上述回归检查；
4. `meta/build-info.txt`：追加父包哈希和本次修复原因。

重新封装时已用递归字节比较验证 `bin/`、`lib/`、`config/` 与 r4 完全一致，并验证
`meta/image-id.txt`、`meta/sdk2-commit.txt`、`meta/symlinks.txt` 未变。通过的离线验证：

```text
H2_GATE_SCHEMA_OFFLINE_OK
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
UNITREE_H2_LIVE_MOTION_PLAN_OK
```

r5 产物：

```text
D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5.tar.gz
sha256=b1e4d67f400195273a8106310e364aed750c0651044884075c4349d50495d735
```

重新封装工具：

```text
unitreeH2/docker/repackage_stage06d_tty_r5_without_docker.ps1
```

该工具使用 WSL 的 tar/bash，不使用 Docker，也不触碰 PC2。部署 r5 后必须重新运行 r5 的
06C，再运行 r5 的 06D；r4 的 gate 不能跨 release 复用。

## 4. 后续门禁

- 仅 Stage 06D 成功后，才评估 Stage 06E 的一次单轴、短时、非零速度脉冲；首次只能
  使用 `x-positive`，不能直接称为“前进”。
- `writeActionCommand()` 的实机动作测试独立于速度测试，保持锁定，不能在 06D/06E 中
  顺带执行。
- 操作者贴回 06D 完整输出后，将在本文件追加实际命令、远端日志路径、成功/失败标记和
  现场观察结论。

### 4.1 r5 首次 PC2 原生运行记录：06C 通过，06D 停在人工确认口

时间：2026-07-20 16:52:07 至 16:52:19。

计划：在 PC2 原生环境中使用 r5 包重新运行 06C getter-only RPC；06C 通过后进入 06D，
但必须停在人工物理安全确认口，由现场操作者确认后才允许发送零速度和 `StopMove()`。

实际执行：

```text
cd /home/unitree/p2_unitreeH2/build
cd unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5
bash scripts/06_pc2_h2_getters_rpc_gate.sh
bash scripts/07_pc2_h2_zero_stop_gate.sh
```

注意：本轮开头执行 `sha256sum --check --strict` 时失败，原因是本地生成的 `.sha256`
文件写入了 `/mnt/d/Desktop/RoboDog/...` 绝对路径；这不是 PC2 包内容或底层控制链路失败。
本地已将
`unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5.tar.gz.sha256`
修正为便携的 `sha256  文件名` 格式。

06C 实机结果：

```text
LOG=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_20260720_165207_14651.log
H2_GETTER_ONLY_RPC_OK
GETTER_AUDIT_GATE_OK fsm_id=601 fsm_mode=0 mode_value_observation_only=1
H2_STAGE06C_GETTER_RPC_OK fsm_id=601 fsm_mode=0
```

结论：r5 的 06C getter-only RPC 在当前 boot、当前 release、当前 manifest 下通过；
`UnitreeH2 -> LocoClient -> sport` 的只读 RPC 路径仍然读通，当前 FSM 为 `601 HybridWalk`。

06D 当前状态：

```text
LOG=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06d_20260720_165219_14813.log
REQUIRED_GATE_OK=stage06c
Type ZERO_STOP_PHYSICAL_GATES_CONFIRMED exactly:
```

结论：06D 已通过前置检查、manifest、私有库、离线合同和 06C gate 校验，并正确停在人工
物理安全确认提示处。到本记录为止，尚未输入确认短语，因此尚未发送零速度命令，尚未调用
`StopMove()`，也没有执行非零运动。

下一步：只有现场确认吊装/防跌落保护、四个脚轮锁定、测试区清空、第二操作者持遥控器、
并确认原厂停止/阻尼流程后，才输入：

```text
ZERO_STOP_PHYSICAL_GATES_CONFIRMED
```

如果任何一项不满足，必须按 `Ctrl+C` 停止 06D，不继续。

### 4.2 06D 人工确认短语误输入记录

时间：2026-07-20，操作者在普通 shell 提示符下输入：

```text
ZERO_STOP_PHYSICAL_GATES_CONFIRMED
```

实际结果：

```text
ZERO_STOP_PHYSICAL_GATES_CONFIRMED: command not found
```

结论：这不是 H2 底层接口失败，也不是 06D 写入失败；原因是确认短语必须在
`bash scripts/07_pc2_h2_zero_stop_gate.sh` 正在显示
`Type ZERO_STOP_PHYSICAL_GATES_CONFIRMED exactly:` 提示时输入。普通 shell 会把该短语
当成命令执行，因此返回 `command not found`。本次没有发送零速度命令，没有调用 `StopMove()`，
也没有产生机器人运动。

恢复步骤：在 r5 release 目录重新执行 06D，等脚本再次显示人工确认提示后，再输入确认短语。

### 4.3 Stage 06D 实机成功记录

时间：2026-07-20 17:12:55。

实际执行：操作者在 r5 release 目录重新执行 06D，并在脚本提示处输入人工确认短语与现场观察短语。

远端日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06d_20260720_171255_19521.log
```

关键成功标记：

```text
H2_ZERO_STOP_RPC_OK fsm_id=601
COMMAND_RC[zero_stop]=0
GATE_WRITTEN=/home/unitree/p2_unitreeH2/build/h2_control_gate_state/stage06d.ok
H2_STAGE06D_ZERO_STOP_OBSERVED_OK fsm_id=601
```

结论：Stage 06D 已经通过实机验证。通过 `UnitreeH2 -> RobotHardwareInterface`
路径发送了显式零速度，并调用了高层 `StopMove()`；FSM 保持 `601`，未请求任何非零运动，
现场观察记录为 `NO_UNEXPECTED_MOTION_OBSERVED`。这证明 H2 底层适配器的
`writeRobotVelocityCommand()` 零速度停止路径和停止动作链路已经打通。

注意：`StopMove()` 是高层零速度/停止移动命令，不是硬件急停。

### 4.4 r6 准备：修复 06E 的 TTY 检查

检查 06E 脚本时发现，r5 的 `08_pc2_h2_single_axis_motion_gate.sh` 仍然使用
`-t 1` 检查 stdout。由于公共日志函数会把 stdout 重定向到 `tee`，这会导致 06E 在真实
Xshell 交互终端下也可能误判为非 TTY。该问题与此前 r4 的 06D TTY 问题同源。

本地已修复：

```text
scripts/08_pc2_h2_single_axis_motion_gate.sh
scripts/tests/test_h2_gate_schema_offline.sh
```

修复方式：06E 改为检查 `-t 0 && -t 3`，其中 FD 3 是 `h2_prepare_gate()` 在日志
`tee` 重定向前保存的原始 stdout。

已从 r5 重新封装 r6，仅替换 06E 脚本和离线门禁测试；`bin/`、`lib/`、`config/`
与 r5 字节一致。

r6 产物：

```text
D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r6.tar.gz
sha256=0200c41efcd8840103ee3f97b50fc1a759f12c20db42061fe5883186a7cadb64
```

离线验证通过：

```text
H2_GATE_SCHEMA_OFFLINE_OK
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
UNITREE_H2_LIVE_MOTION_PLAN_OK
R6_TTY_FIX_REPACKAGE_HOST_OK
```

因为 gate 文件会绑定当前 release 路径、boot_id 和 manifest，所以 r6 部署到 PC2 后，
必须重新运行 r6 的 06C 和 06D，再运行 r6 的 06E；不能复用 r5 的 `stage06d.ok`。

### 4.5 Stage 06E 实机首次单轴非零速度脉冲记录

时间：2026-07-20 17:34 至 17:37 后。

最终成功日志：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06e_20260720_174024_26598.log
```

同一 r6 release 的 06C、06D 日志和 manifest：

```text
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_20260720_173417_24452.log
/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06d_20260720_173658_25282.log
manifest_sha256=5fa8b528ba5fca1727b5d6cea7b18597d531f82594697807af0a34e2f0631425
```

实际执行：操作者在 r6 release 目录完成 06C、06D 后执行首次 06E：

```text
bash scripts/08_pc2_h2_single_axis_motion_gate.sh x-positive
```

06E 计划输出：

```text
H2_LIVE_PLAN axis=x-positive vx=0.050 vy=0.000 omega=0.000 vendor_duration_s=0.250 local_pulse_ms=120 watchdog_ms=180 expected_fsm=601
H2_LIVE_PRINT_PLAN_ONLY_NO_DDS
```

关键实机执行标记：

```text
H2_LIVE_STATE fsm_id=601 fsm_ret=0 fsm_mode=0 mode_ret=0 mode_value_observation_only=1
H2_LIVE_PULSE_BEGIN axis=x-positive
H2_LIVE_PULSE_RPC ret=0
H2_LIVE_STOP reason=pulse-complete stop_move_ret=0 zero_velocity_ret=0
H2_LIVE_FINAL_STATE fsm_id=601 fsm_ret=0 stop_ret=0
H2_LIVE_SINGLE_AXIS_PULSE_OK axis=x-positive observed_direction_must_be_recorded=1
COMMAND_RC[single_axis_motion]=0
GATE_WRITTEN=/home/unitree/p2_unitreeH2/build/h2_control_gate_state/stage06e.ok
H2_STAGE06E_SINGLE_AXIS_PULSE_OK axis=x-positive observed=direction unclear
```

现场观察：操作者反馈“几乎没动”，脚本中记录为 `direction unclear`。

结论：Stage 06E 已证明 `writeRobotVelocityCommand()` 的非零速度路径在 H2 实机上完成一次
`SetVelocity()` RPC，返回 `ret=0`，并随后成功执行停止收尾；FSM 保持 `601`。由于本次
脉冲极小，计划速度仅 `0.050 m/s`、本地脉冲约 `120 ms`，理论位移量只有毫米级，再叠加
保护架/脚轮锁定/原厂高层运动平滑等因素，肉眼几乎看不到运动是合理结果。本次不能用于
标定 `x-positive` 对应的实际前后方向，只能记录为“非零速度链路已打通，物理方向未确认”。

后续方向观察已实现为 r7 可配置 Stage 06E，不直接手工修改 r6。r7 继续保持单轴、短时、
人工确认、遥控器待命和 `StopMove()` 收尾；实现、哈希、离线验收和部署命令见
`Unitree_H2_Stage06E_r7可配置脉冲准备与离线验收记录_20260720.md`。
