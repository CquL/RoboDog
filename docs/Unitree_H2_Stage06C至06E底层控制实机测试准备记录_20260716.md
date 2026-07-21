# Unitree H2 Stage 06C 至 06E 底层控制实机测试准备记录

日期：2026-07-16（Asia/Shanghai）

## 1. 本轮结论

本轮已经把 Unitree H2 的 `robot_hardware` 抽象接口、实机测试程序、PC2 分级门禁和自包含运行包收紧并完成离线验收。

当前可以进入的下一步只有 **Stage 06C getter-only Loco RPC 实机验证**。截至本文写入时：

- 没有把本轮 `r4` 包上传到 PC2；
- 没有调用 H2 的 getter RPC；
- 没有写入零速度或 `StopMove()`；
- 没有发送任何非零速度；
- 没有调用 `Start`、`StandUp`、`Damp`、`Squat`、`Sit`；
- 没有停止或修改原厂 `sport`、DDS、网络或 PC2 系统服务；
- 没有在 PC2 安装 Docker，也没有覆盖 PC2 的 `/opt/unitree_robotics`。

因此，离线成功不能描述为“实机已经可以运动”。

## 2. 抽象层与 H2 SDK2 的实际映射

`RobotFactory::RobotAllocate(YAML::Node node)` 仍保留在 `include/robot_factory.h` 中，分配逻辑是原来的头文件内联形式。新增的分支是：

```cpp
}else if(robot_type == "unitree_h2"){
    return std::make_shared<UnitreeH2>(node);
}
```

当前没有 `ROBOT_HARDWARE_HAS_UNITREE_H2`、`BUILD_UNITREE_H2_ADAPTER` 或 `src/robot_factory.cpp`。

接口映射如下：

| 抽象接口 | Unitree H2 SDK2 调用 | 说明 |
|---|---|---|
| `initRobotHardware()` | `ChannelFactory::Init(...)`、`h2::LocoClient::Init()`、`SetTimeout()`，按配置执行 `GetFsmId()` | 初始化 DDS/RPC 客户端；允许运动时必须处于 FSM 601 |
| `writeRobotVelocityCommand(cmd)` | `LocoClient::SetVelocity(vx, vy, omega, duration)` | 上层仍只调用抽象接口；不是发布 ROS 2 控制 topic |
| `writeActionCommand(ACTION_STOP_MOVE)` | `LocoClient::StopMove()` | 高层零速度封装，不是硬件急停 |
| `writeActionCommand(stand_up/prepare_motion/damp/squat/sit)` | `StandUp/Start/Damp/Squat/Sit` | 已映射，但默认配置和本轮实机门禁禁止调用 |
| `writeActionCommand(lie_down/未知动作)` | 不进入 SDK | 在适配层直接拒绝 |

这里实现的是“不依赖 ROS 2 节点/API 的上层控制接口”，但 Unitree SDK2 内部传输仍然是 CycloneDDS RPC。换言之，算法和机器人厂商控制 API 已通过 HAL 解耦，网络传输并不是“完全没有 DDS”。

固定使用的官方 SDK2 快照：

- 仓库：`unitreerobotics/unitree_sdk2`
- commit：`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`
- 本地元数据：`unitreeH2/vendor/unitree_sdk2/.source.json`

## 3. 本轮新增的控制安全保护

### 3.1 HAL 内保护

- 非零速度使用精确非零判断，极小非零值不能绕过运动门禁；
- 绝对上限：`|vx| <= 0.20`、`|vy| <= 0.10`、`|omega| <= 0.30`、SDK duration `<= 0.30 s`；
- 允许运动的初始化必须读到 `GetFsmId() == 601`；
- 每次非零速度调用都会在命令互斥锁内再次读取 FSM，并要求仍为 601；
- watchdog 在调用 `SetVelocity` 前置为 active；
- `SetVelocity` 返回错误或抛异常时立即尝试 `StopMove`；若停止失败，watchdog 保持运行并继续重试；
- 对“命令可能已经送达、客户端却收到错误”的歧义响应，析构路径仍执行 best-effort `StopMove`。

### 3.2 单轴实机程序保护

首轮只允许未标定轴名，不允许直接写“前进、后退、左移、右移”：

| 轴名 | 计划命令 |
|---|---:|
| `x-positive` / `x-negative` | `vx = +/-0.05` |
| `y-positive` / `y-negative` | `vy = +/-0.03` |
| `yaw-positive` / `yaw-negative` | `omega = +/-0.08` |

统一参数：SDK duration `0.25 s`、本地非零脉冲 `120 ms`、watchdog `180 ms`、最终零速保持 `1000 ms`。

另外新增：

- 5 秒倒计时实时输出，不再被 shell 命令替换缓存；
- 倒计时结束后再次执行零速度和 `StopMove()`，再读取 FSM/mode，然后才允许非零 RPC；
- `SIGINT`、`SIGTERM`、`SIGHUP`、`SIGQUIT` 都进入停止路径；
- 静态确认字符串本身不再足够：06E launcher 会生成 64 位十六进制一次性 token 文件；live binary 在 DDS 初始化前验证路径、owner、mode 0600、link count 和内容，并删除该文件；
- 一次性文件只用于降低误调用风险。同一 Unix 用户可修改自己的脚本/二进制，因此在不增加 root-owned broker 或独立服务账号的前提下，不能宣称它对恶意同用户具有密码学不可绕过性。

### 3.3 06C/06D/06E 门禁链

- gate 文件必须是普通文件、非符号链接、owner 为 `unitree`、mode 为 0600；
- 字段数量、字段顺序、stage、hostname、boot ID、release、manifest 和成功语义必须匹配；
- 06D 保存并验证父 `stage06c.ok` 的 SHA256；
- 06E 保存并验证父 `stage06c.ok` 与 `stage06d.ok` 的 SHA256；
- 重新完成 06C 会删除旧的 06D/06E gate；重新完成 06D 会删除旧的 06E gate；
- 第一次 06E 必须是 `x-positive`；已有 06E 历史文件如果畸形会 fail-closed；
- 新增纯离线畸形 gate 回归：`H2_GATE_SCHEMA_OFFLINE_OK`。

## 4. 离线构建与验收证据

### 4.1 最终候选镜像

- tag：`unitree_h2:amd64-live-test-candidate`
- exact image ID：`sha256:9a7fd813d6cb509efebb8064bae47613a196a64c9490fdee26270f2b3fd12035`
- architecture：`amd64`
- SDK2 commit：`21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`
- 构建方式：`--pull=false --network=none --no-cache`
- inherited entrypoint：已清空
- `RMW_IMPLEMENTATION`：未设置
- HAL 动态依赖：没有 `librcl*`、`librmw*` 或 `libros*`

镜像仍继承 `jezetek:navigation_system_amd64` 的 Ubuntu/ROS 基础环境，所以准确表述是“HAL 二进制无 ROS 动态库依赖”，不能表述为“整个镜像完全没有 ROS 文件”。

无缓存构建中的 6 项 CTest 全部通过：

1. `unitree_h2_factory_contract_test`
2. `unitree_h2_direct_api_contract_test`
3. `unitree_h2_live_motion_plan_test`
4. `unitree_h2_probe_rejects_conflicting_modes`
5. `unitree_h2_live_motion_rejects_missing_interlocks`
6. `unitree_h2_live_motion_rejects_static_ack_without_one_time_gate`

结果：`100% tests passed, 0 tests failed out of 6`。

### 4.2 最终 PC2 原生包

- 名称：`unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz`
- 路径：`D:/Desktop/RoboDog/unitreeH2/runtime_bundle/`
- 大小：`8,566,238 bytes`
- archive SHA256：`ddbff7ae8012c745daa175517f2e16ceb9605c97045cf60ac2068d43a5639b60`
- manifest SHA256：`d8f1170b40a73e21ef7ded35b5a04ef7fe0a3bc97ed69f8cb6e3b82f63e79ebe`
- build time：`2026-07-16T13:12:39+00:00`
- PC2 Docker required：`false`

包内包含：

- `robot_test_unitree_h2`
- `robot_test_unitree_h2_live_motion`
- 三个 H2 离线合同测试程序
- 私有 `librobot_hardware.so`
- 私有 `libddsc.so` / `libddscxx.so` 与 SONAME 链接
- 私有 `libyaml-cpp.so.0.7.0` 与 SONAME 链接
- 当前整体 `librobot_hardware.so` 仍需要的 `libmc_sdk_zsl_1_x86_64.so`
- `unitree_h2.yaml`
- 06C/06D/06E/common gate 脚本和 gate schema 离线测试
- image ID、SDK commit、build info、symlink 清单和逐文件 manifest

H2 控制路径没有调用 ZSL API，但整体共享库仍携带 ZSL DSO，这是后续按厂商拆分链接依赖的技术债，不影响本轮 H2 SDK2 ABI 验收。

完整包验收在 `--network none`、`--read-only`、`--cap-drop ALL`、`no-new-privileges` 环境下完成，且 `/verify` 显式挂载为 `exec`。通过标记：

- `H2_GATE_SCHEMA_OFFLINE_OK`
- `UNITREE_H2_FACTORY_CONTRACT_OK`
- `UNITREE_H2_DIRECT_API_CONTRACT_OK`
- `UNITREE_H2_LIVE_MOTION_PLAN_OK`
- `H2_PC2_NATIVE_BUNDLE_OFFLINE_OK`
- `H2_PC2_NATIVE_BUNDLE_HOST_GATE_OK`

### 4.3 被淘汰的产物

以下产物禁止上传 PC2：

| 产物 | SHA256 | 原因 |
|---|---|---|
| `r2.tar.gz` | `fa923c5dc9f8ef92ef49f9314c3f266e111ddea080d6e25ab24b306b4c779a28` | 初始 verifier 的 tmpfs 默认 `noexec` 导致 `ldd` 误报；随后源码和安全门禁又更新，内容已过期 |
| `r3.tar.gz` | `e59bb466cfd14fc6da2f7aa999d13f073ea544088280002da868d6fa6225eeed` | 新内容已生成，但 Windows PowerShell 把合同测试的预期 stderr 诊断当成终止异常，完整 host gate 未跑完 |

历史镜像 `sha256:696f2fb1...` 也早于本轮安全修改，不再是当前 live-test candidate。

## 5. PC2 已知环境与部署方式

此前只读盘点确认：

- hostname：`unitree-H2-pc2`
- OS：Ubuntu 22.04.4 x86_64
- PC2 `eth0`：`192.168.123.162/24`
- PC1：`192.168.123.161`
- 项目根目录：`/home/unitree/p2_unitreeH2/`
- CycloneDDS XML：`/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml`
- XML SHA256：`bc977aacd0e44804cb8da24d24d6fe5ed654aad9ed7b15ed3ef36d32f27a1796`
- PC2 盘点时没有 Docker；本轮不安装 Docker。

PC2 旧 `/opt/unitree_robotics` 不具备本轮 H2 Loco 完整头文件/归档组合，因此不能把本地新头文件与 PC2 旧 `libunitree_sdk2.a` 混用。`r4` 使用同一候选镜像内构建的程序和私有 DDS 动态库，解压到：

`/home/unitree/p2_unitreeH2/build/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4/`

它不会写 `/opt`，不会替换原厂 SDK，不要求 PC2 Docker。

## 6. 下一步执行顺序

### 6.1 上传并只执行 Stage 06C

Windows PowerShell：

```powershell
scp "D:/Desktop/RoboDog/unitreeH2/runtime_bundle/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz" "D:/Desktop/RoboDog/unitreeH2/runtime_bundle/unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz.sha256" unitree@192.168.123.162:/home/unitree/p2_unitreeH2/build/
```

PC2 的 `unitree` 用户终端：

```bash
cd /home/unitree/p2_unitreeH2/build
sha256sum --check --strict unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz.sha256
tar -xzf unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4.tar.gz
cd unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4
bash scripts/06_pc2_h2_getters_rpc_gate.sh
```

06C 会向 `/api/sport/request` 发布 getter RPC 请求，因此它不是网络意义的纯接收；但程序只调用 `GetFsmId`、`GetFsmMode`、`GetAvailableFsmIds`，不调用控制 setter、不自动切 FSM。成功标记应为：

```text
H2_STAGE06C_GETTER_RPC_OK fsm_id=601 ...
```

完成后把最新日志复制回 Windows：

```powershell
scp unitree@192.168.123.162:/home/unitree/p2_unitreeH2/logs/h2_pc2_stage06c_*.log "D:/Desktop/RoboDog/unitreeH2/remote/"
```

应先审计 06C 日志，再决定是否进入 06D。

### 6.2 Stage 06D：零速度与 StopMove

只有 06C 成功，并且现场确认以下五项后，才可在交互式终端运行 06D：

1. 使用官方防坠/保护支架；
2. 支架四个脚轮全部锁定；
3. 测试区域净空，人员不在机器人可达范围；
4. 第二操作员手持原厂遥控器；
5. 已确认交付 H2 的高层停止/阻尼处置流程。

命令：

```bash
bash scripts/07_pc2_h2_zero_stop_gate.sh
```

该阶段会真实写入零速度并调用 `StopMove()`，但不会写非零速度。观察员必须明确确认没有异常运动。`StopMove()` 不是硬件急停，不能代替原厂遥控器和防坠设施。

### 6.3 Stage 06E：一次极小单轴脉冲

只有 06C、06D 同一 bundle、同一 boot、父 gate 哈希全部通过，而且五项物理门禁再次确认后，才运行：

```bash
bash scripts/08_pc2_h2_single_axis_motion_gate.sh x-positive
```

程序只执行一次 `x-positive` 极小脉冲，不循环其他方向。必须现场记录真实物理方向，确认安全后才能决定后续 `x-negative/y/yaw` 测试。方向未标定前不得把 `x-positive` 写成“前进”。

## 7. 本轮文件与日志

主要源码：

- `robot_hardware/robot_hardware/include/unitree/unitree_h2.h`
- `robot_hardware/robot_hardware/src/unitree/unitree_h2.cpp`
- `robot_hardware/robot_hardware/robot_test_unitree_h2.cpp`
- `robot_hardware/robot_hardware/robot_test_unitree_h2_live_motion.cpp`
- `robot_hardware/robot_hardware/tests/unitree_h2_direct_api_contract_test.cpp`
- `robot_hardware/robot_hardware/tests/unitree_h2_live_motion_plan_test.cpp`

PC2 gate：

- `unitreeH2/remote/h2_pc2_hal_gate_common.sh`
- `unitreeH2/remote/06_pc2_h2_getters_rpc_gate.sh`
- `unitreeH2/remote/07_pc2_h2_zero_stop_gate.sh`
- `unitreeH2/remote/08_pc2_h2_single_axis_motion_gate.sh`
- `unitreeH2/remote/tests/test_h2_gate_schema_offline.sh`

可重复构建/打包：

- `unitreeH2/docker/build_live_test_candidate.ps1`
- `unitreeH2/docker/build_stage06c_pc2_native_bundle.ps1`

实际日志：

- `unitreeH2/runtime_bundle/build_live_test_candidate_20260716_r3_nocache.log`
- `unitreeH2/runtime_bundle/build_stage06c_pc2_native_bundle_20260716_r4.log`

关键源码 SHA256：

- `unitree_h2.h`：`a166e215c5d5f9365f0ec1949aebafa5f1e860b8c1ca3343701979bcfd6162f7`
- `unitree_h2.cpp`：`2cf31885ac20227a369ec186626cc1402ff3b648a072bd35c4f439792463acfc`
- live motion harness：`6e008de0e9eee263b3bb58da28efb7f236477158a1556874d49bf4616e6b3bf1`
- common gate：`550abdc0afed72d962b688c36d0cee29611ff2293e661d175ebcf0b251745538`
- 06C：`f00f51b87480414aa9a1d25f3fa46ee0b7e98ec07e22ac5d3b9d5e1a38efee36`
- 06D：`e0e7480cfd5256a5100de0e7a9503ec6f6f5efaa90d51cb9c4092fe5a2a17057`
- 06E：`5a87135f90d62eb1dce836cea81d3980ecb3330a4fcfb0f41d9ec8019573049c`

## 8. 仍未完成的范围

- Stage 06C/06D/06E 实机结果仍为空；
- Stage 06B 生产型 `unitree_h2_state_source` 尚未完成；
- 导航传感器驱动、点云/里程计标准化、导航算法和正式 `h2_runtime` 尚未接入；
- H2 传感器容器不能把 X30 的 `x30_livox_ros2_transfer` 整体照搬；
- 本轮只验证了用户明确要求的直接 HAL 控制链准备，不代表完整算法 Docker 已可迁移运行；
- 外部并发 Loco DDS 客户端仍可能在进程检查之间竞争写控制；本轮通过最后零速、三次 FSM 检查、单脉冲、watchdog 和现场遥控器降低风险，但没有修改原厂服务来做全局控制所有权仲裁。
