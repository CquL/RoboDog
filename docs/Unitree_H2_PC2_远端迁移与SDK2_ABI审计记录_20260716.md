# Unitree H2 PC2 远端迁移与 SDK2 ABI 审计记录

日期：2026-07-16  
状态：远端目录迁移和阶段 04 纯 ABI 审计完成；阶段 05 只读 HG 订阅探针已在本地完成复审，等待 PC2 实际执行  
安全边界：本轮没有发送 topic、RPC、速度、动作或 FSM 命令，没有停止原厂进程，也没有运行 H2 HAL

## 1. 本轮计划与实际完成

计划：

1. 验证旧审计文件是否只迁入 `/home/unitree/p2_unitreeH2/`；
2. 核对 PC2 工具链、SDK2 archive、HG IDL、DDS 动态库和 ABI；
3. 只有 ABI 门槛通过后，才决定是否允许纯订阅探针进入 PC2；
4. 继续禁止 H2 Loco、`GetFsmId()`、HAL、零/非零速度和动作测试。

实际：迁移成功，纯 ABI 审计成功；同时发现 PC2 缺少 HG
`SportModeState_.hpp` 和全部 H2 Loco 头文件。原阶段 05 草案因此不能原样运行，现已
移除 SportModeState，收缩为 7 个只读状态 topic，并增加完整哈希、二进制发送路径、
超时和残留进程门槛。

## 2. 回传日志完整性

| 日志 | 字节 | 行数 | SHA256 | 结束标记 |
|---|---:|---:|---|---|
| `h2_pc2_workspace_migration_20260716.log` | 3,278 | 35 | `B3DEE2ADF32F67130FCF4A26B1D6C40F17D3E67E092CF29F8352F78D0E6C73E1` | `P2_UNITREE_H2_WORKSPACE_MIGRATION_OK` |
| `h2_pc2_sdk2_abi_audit_20260716.log` | 103,199 | 960 | `A33CDDC041C292E4B6FEAFD41E5CFBA8E93493E187760A5A0072F49D938502C0` | `READ_ONLY_PC2_SDK2_ABI_AUDIT_OK` |

阶段 04 没有编译、没有初始化 DDS、没有创建 reader/writer，也没有调用 RPC。

## 3. 远端迁移核验

- `00`–`03` 四个脚本已从 `/home/unitree/` 移到
  `/home/unitree/p2_unitreeH2/scripts/`；四份既有日志已移到 `logs/`。
- `04_pc2_sdk2_abi_read_only_audit.sh` 原本已直接放在新目录，远端哈希与本地一致。
- 清单没有 `SKIP_MISSING`、目标冲突或 shell 错误；没有证据表明清单外文件被移动。
- 迁移日志第 15、28 行是在 `tee` 尚未结束时对自身取哈希，只能视为中间值；上表是
  回传完成后的最终大小和哈希。
- 远端 `02_pc2_ros2_graph_read_only_audit.sh` 仍是执行时的旧版：1,981 bytes，SHA256
  `78712811...CA935`。本地修正版为 2,107 bytes，SHA256
  `339F2165C8DE93BBF4FB901F7C6B6A27323894C84F1252D31E6529FEAE1BA5A6`；下一次上传时只
  覆盖远端脚本留档，不需要重跑图审计。

## 4. PC2 SDK2/ABI 结论

### 4.1 通过项

- PC2 是 x86_64 Ubuntu 22.04；CMake 3.22.1、GCC/G++ 11.4.0、glibc 2.35 和
  libstdc++ 12.3.0 可用。
- `/opt/unitree_robotics/lib/libunitree_sdk2.a` 是 128 个 x86-64 对象组成的静态库，
  27,362,488 bytes，SHA256
  `e436eedf0d81e9efa10b039f8151743f46547535a99790ed19ddacc105098cd4`。
- CMake 包版本为 `2.0.0`，包含 `ChannelFactory::Init(int, string)` 和 `Release()`。
- HG `LowState`、`IMUState`、`BmsState`、`MainBoardState` 头文件以及
  `channel_factory.hpp`、`channel_subscriber.hpp` 与本地固定 SDK 快照逐字节一致。
- `/unitree/opt` 与 `/opt/unitree_robotics` 的 `libddsc`、`libddscxx` 分别逐字节一致；
  共享库最高观测需求为 `GLIBC_2.17`、`GLIBCXX_3.4.26`、`CXXABI_1.3.9`，低于 PC2
  运行时能力。

### 4.2 未通过项

- `/opt/unitree_robotics/include/unitree/idl/hg/SportModeState_.hpp` 缺失。
- 在 `/opt/unitree_robotics`、`/usr/local`、`/unitree/opt`、`/home/unitree` 均未找到
  `h2_loco_api.hpp` 或 `h2_loco_client.hpp`。
- PC2 静态库与本地固定官方快照的大小和 SHA256 不同。禁止把本地
  SportModeState/H2 Loco 头或本地库单独复制到 PC2 混合链接。
- 静态库自身没有输出可用的最高 GLIBC/CXXABI 版本记录；最终兼容性仍要由 PC2 本机
  实际编译、链接和 `ldd` 证明。

因此：PC2 工具链和 4 类 HG 状态的 native 订阅基础通过；完整 H2 Loco/HAL 控制 ABI
没有通过，`RobotHardwareInterface` 的实机速度和动作控制仍不可用。

## 5. 阶段 05 最终只读探针

| 文件 | 字节 | SHA256 |
|---|---:|---|
| `remote/05_pc2_build_run_hg_state_read_only.sh`（r2） | 8,730 | `4D47D53CA1B101FBB671686D130061F92AA6250E0A15BB8991E47A5FE0465808` |
| `remote/05_hg_state_probe/CMakeLists.txt` | 510 | `0D9D32404AD833ACC25EBA899AD345899F599CA2EE710A22E0014EA1DD84D492` |
| `remote/05_hg_state_probe/h2_hg_state_read_only_probe.cpp` | 12,167 | `D794406DFBEB4C88C1256BF24DCABA6AE3C5C9B504B10313DE0DE90A097BE844` |

只读 topic 白名单：

```text
rt/lowstate_raw
rt/lowstate
rt/lf/lowstate
rt/secondary_imu
rt/lf/secondary_imu
rt/lf/bmsstate
rt/lf/mainboardstate
```

成功条件是三个 LowState 候选至少一个、两个 secondary IMU 候选至少一个，并且 BMS、
MainBoard 均收到样本。SportModeState 暂不测试。

Runner 在 DDS 初始化前固定检查源码、PC2 archive、4 个 HG 头、Channel 头、三份 CMake
合同、DDS 库和 CycloneDDS XML 的 SHA256；使用全新构建目录。运行前还会检查 `eth0`
为 UP 且地址是 `192.168.123.162/24`。构建后必须满足：

- `ldd` 将 `libddsc.so.0`、`libddscxx.so.0` 解析到 `/opt/unitree_robotics/lib`；
- `nm` 不含 `CreateSendChannel`、`ChannelPublisher`、控制 Client、LowCmd/MotorCmd、
  Request 或 DDS write 符号；
- 二进制只含上述 7 个 `rt/` 字符串；
- 15 秒采样由 25 秒 timeout 和额外 5 秒强制终止保护包围；结束后不得残留进程。

准确的 DDS 边界是：应用没有 DataWriter、发送通道或控制 Client。SDK2
`ChannelFactory::Init` 内部会预建 DDS Publisher 容器，因此不能写成“完全没有
Publisher 实体”，但没有 `SetWriter/CreateSendChannel`，不会形成应用发送路径。

## 6. 本地离线复核

新增 `unitreeH2/docker/verify_hg_state_probe_offline.sh`。在
`unitree_h2:amd64-offline` 中使用 `--network none`、只读根文件系统、drop all
capabilities 和 `/tmp` tmpfs 执行后：

```text
ELF64 x86-64 build=OK
offline binary SHA256=B5B75863B3D98B0A78969F08DD15CE0B25D26D5D0DE5EB57F8C6F9B5EF228C3C
forbidden send symbols=none
rt topic allowlist=exactly 7
STAGE05_HG_STATE_PROBE_OFFLINE_SAFETY_OK
```

该二进制来自本地不同 SDK，只用于源码语义和安全复核，绝不能复制到 PC2。PC2 必须
本机编译。

## 7. 当时的 Stage 05 执行顺序

1. 把本地修正版 `02` 覆盖到远端 `scripts/`，只作版本归档，不重跑。
2. 只上传阶段 05 runner、CMake 和 C++ 三个文件，不上传任何 vendor 头或库。
3. 在 PC2 运行一次 runner，将完整输出写入新的 stage-05 日志并回传。
4. 只有日志证明 PC2 编译、`ldd`、二进制门槛、样本计数、频率和清理全部通过，才开始
   设计 `unitree_h2_sensor_bridge`；此时仍不调用 `GetFsmId()` 或 H2 HAL。

当前明确禁止：H2 Loco/RPC、`GetFsmId()`、`--zero-stop`、非零速度、动作/FSM、
`rt/lowcmd`、`/arm_sdk`、停止原厂节点，以及复用 X30 控制或传感器停启脚本。

## 8. Stage 05 后续实际结果

- r1 编译成功，但因 `lib/x86_64/` 尚未写入允许路径，在运行前以
  `STAGE05_RUN_RC=14` 安全退出。
- r2 runner SHA256：
  `4D47D53CA1B101FBB671686D130061F92AA6250E0A15BB8991E47A5FE0465808`。
- r2 在七 topic 二进制接收白名单通过后完成 15 秒纯订阅，最终
  `READ_ONLY_PC2_HG_STATE_PROBE_OK`、`STAGE05_RUN_RC=0`。
- 高频 `rt/lowstate`/`rt/secondary_imu` 约 1,048 Hz；四个低频 `rt/lf/*`
  状态约 20.158 Hz；`rt/lowstate_raw` 无样本。
- Stage 06A Docker 离线自包含审计随后已在本地通过：2/2 CTest、Factory、直接 API
  fake-SDK 合同和镜像动态库检查均成功，镜像 digest 为
  `sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa`。
- 下一门禁不是 `GetFsmId()` 或控制，而是 Stage 06B 镜像内生产型状态源纯订阅；PC2
  当前未安装 Docker，本轮未安装。

完整 r1/r2 证据、直接 HAL 控制边界和后续门禁见
`docs/Unitree_H2_直接HAL控制与Docker解耦架构记录_20260716.md`。
Stage 06A 的代码改动、测试覆盖与产物哈希见
`docs/Unitree_H2_HAL直接接口与Stage06A离线验收记录_20260716.md`。
