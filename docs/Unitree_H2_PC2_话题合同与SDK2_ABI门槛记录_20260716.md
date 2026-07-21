# Unitree H2 PC2 话题合同与 SDK2 ABI 门槛记录

日期：2026-07-16  
输入：`unitreeH2/remote/h2_pc2_topic_contract_20260716.log`、
`h2_pc2_sdk2_abi_audit_20260716.log`  
状态：阶段 04 纯 ABI 审计通过；阶段 05 已收缩并通过本地安全复核，等待 PC2 原生只读订阅测试  
安全边界：没有发布 topic、调用 RPC/service、发送 action、初始化 H2 HAL、发送零/非零
速度、切换 FSM、写入 `/lowcmd` 或 `/arm_sdk`。

## 1. 日志完整性

```text
size=29528 bytes
lines=1088
SHA256=1D8428F6910A23C6E85994AA633186C24C14A42ADA8B8618FFE76B15CB384021
end marker=READ_ONLY_PC2_TOPIC_CONTRACT_AUDIT_OK
```

上轮遗留的 ROS 2 CLI daemon 已成功停止，`COMMAND_RC=0`；脚本结束时再次确认
`ROS2_CLI_DAEMON_NOT_RUNNING`。没有停止任何原厂机器人节点或服务。

## 2. H2 高层服务端点

`/api/sport` 已从“计数存在”推进到“端点成对确认”：

- request 的唯一 subscriber 与 response 的唯一 publisher 具有同一 DDS participant
  GID 前缀 `01.10.57.65.e1...`，可判定为同一个 sport 服务端参与者；
- 另外四个 participant 各有 request publisher 与 response subscriber，是四组客户端
  候选；
- publisher 均为 Reliable，reader 为 Best Effort 或 Reliable，当前未发现 QoS 不兼容。

这仍然没有发送任何请求，不能证明 API 7001 `GetFsmId()` 能往返，也不能从 GID 确认
服务端所在 IP/进程。禁止用 `ros2 topic pub` 手工拼装 RPC。

## 3. H2/HG 本体状态合同

PC2 当前 ROS 2 工作空间没有 `unitree_hg` 包，相关 `pkg prefix` 与 `interface show`
均返回 1；因此 ROS CLI 不能直接解码 HG 状态。

但 PC2 原生 SDK2 安装中存在：

```text
/opt/unitree_robotics/include/unitree/idl/hg/LowState_.hpp
/opt/unitree_robotics/include/unitree/idl/hg/IMUState_.hpp
/opt/unitree_robotics/include/unitree/idl/hg/BmsState_.hpp
```

因此正确方向是使用 PC2 同一套 SDK2 headers + archive 编译纯 subscriber 探针，而不是
先安装一个未知版本的 ROS `unitree_hg` 包。

已确认的 HG publisher：

| ROS 2 图名称 | 类型 | Publisher QoS | 当前状态 |
|---|---|---|---|
| `/lowstate_raw` | HG LowState | Reliable/KeepLast 1/Volatile | 端点存在，未取样 |
| `/lf/lowstate` | HG LowState | Reliable/KeepLast 1/Volatile | 端点存在；同名还混有 GO subscriber |
| `/secondary_imu` | HG IMUState | Reliable/KeepLast 1/Volatile | 端点存在，未取样 |
| `/lf/secondary_imu` | HG IMUState | Reliable/KeepLast 1/Volatile | 端点存在，未取样 |
| `/lf/bmsstate` | HG BmsState | Reliable/KeepLast 1/Volatile | 端点存在，未取样 |
| `/lf/mainboardstate` | HG MainBoardState | Reliable/KeepLast 1/Volatile | 端点存在，未取样 |
| `/sportmodestate` | HG SportModeState | Reliable/KeepLast 1/Volatile | 可作为只读 FSM 状态候选 |

上述低状态、IMU、BMS、主板 publisher 使用相同 participant GID 前缀
`01.10.e8.c9...`，说明它们来自同一 DDS 参与者；这只是拓扑关联，不能据此断言主机
或进程身份。

`/lf/emergency_stop` 没有 publisher，只有一个 subscriber。它不是当前可读取的急停
状态，不能纳入安全联锁，也禁止在协议未知时向其发布。

## 4. 标准 ROS 2 输入活性与质量

| Topic | 发布者/QoS | 观测 | 当前决定 |
|---|---|---|---|
| `/dog_imu_raw` | `dog_control_pub`，Reliable/KeepLast 1000 | 约 500 Hz | 仅作兼容层姿态候选 |
| `/dog_odom` | `dog_control_pub`，Reliable/KeepLast 1000 | 约 490–501 Hz | 暂列里程计候选 |
| `/point_in_map` | `QtServer`，Reliable/KeepLast 10 | 10 秒无样本 | 不认定雷达/地图已工作 |
| `/tf` | `dog_control_pub`，Reliable/KeepLast 100 | 10 秒无样本 | TF 合同未成立 |
| `/frontvideostream` | 无 publisher | 无输入 | 相机合同未成立 |

`/dog_imu_raw` 样本时间为 2026-07-16 11:57:54+08:00，frame 为
`dog_imu_link`。四元数有值，但 angular velocity 与 linear acceleration 三轴全部为
0，相关协方差也不完整。它不能作为导航所需的完整原始 IMU；优先读取 HG
`secondary_imu`。

`/dog_odom` 样本时间为 2026-07-16 11:57:59+08:00，frame 为
`odom -> robot_center`，pose/twist 有值，但两组 covariance 全为 0，坐标原点、重置
规则、漂移和与机体 frame 的关系尚未确认，不能直接按高可信融合输入使用。

## 5. 现役危险写入面

- `/lowcmd` 有 HG publisher 与 HG subscriber，已形成活动控制链；
- `/arm_sdk` 有 HG publisher 与两个 HG subscriber；
- `/sportmodestate` 所在 DDS participant 还发布 `/lowcmd` 并订阅 `/arm_sdk`，说明
  现有运动状态/低层控制链正在工作，不能加入任何“零值测试”publisher；该 participant
  与 `/api/sport` RPC 服务端的 GID 前缀不同，不能混为同一进程；
- H2 适配器继续只设计为高层 Loco RPC，不接管这些低层 topic。

## 6. PC2 SDK2 版本差异与决定

PC2：

```text
/opt/unitree_robotics/lib/libunitree_sdk2.a
size=27362488
SHA256=e436eedf0d81e9efa10b039f8151743f46547535a99790ed19ddacc105098cd4
H2 Loco headers=未发现
```

Windows 本地固定官方快照 `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`：

```text
vendor/unitree_sdk2/lib/x86_64/libunitree_sdk2.a
size=27666376
SHA256=08402AEA74150DFBFC3FBFDED4CA746916A8D892B54D2BADE0CBF392A3BE4029
H2 Loco headers=存在
```

两个 archive 大小和哈希不同。不得只复制本地较新的 H2 Loco 头文件，再链接 PC2 的
旧 archive；headers、archive 和 DDS 依赖必须来自同一 SDK 快照。

另一方面，PC2 `/unitree/opt` 与 `/opt/unitree_robotics` 的两套 `libddsc`、
`libddscxx` 分别字节相同：

```text
libddsc    SHA256=4038630c231412f7b34a2ea60df192bbdebd0a57f22ac7f35c1b6d28323e695c
libddscxx  SHA256=d3e7c1b03123c2745839f2465041777ded090ad66d62f1372949209254f7ebe5
```

所以当前确认的是“路径重复但二进制一致”，不是 DDS 库冲突。

## 7. PC2 远端目录规范

用户指定后续统一使用：

```text
/home/unitree/p2_unitreeH2/
├── scripts/
├── logs/
├── src/
├── build/
└── config/
```

只迁移本项目明确列出的 00–03 脚本和四份 `h2_pc2_*.log`，不触碰
`graph_pid_ws`、`slam_config`、原有 `p2`、NoMachine、Anaconda、隐藏文件或其他用户
文件。迁移脚本：

```text
pc2_workspace_migrate_to_p2_unitreeH2.sh
SHA256=B723C5988FB63291D6DB7FCA52789999752F3B3130C10749B1FE433A057C9391
```

## 8. 阶段 04：只读 ABI 审计（已完成）

已执行脚本：

```text
04_pc2_sdk2_abi_read_only_audit.sh
SHA256=F975EA1CE8DE356DAE9FDFDDCADAA094CA3E0E9CA557CF3B076C5922889D3A42
```

日志为 103,199 bytes、960 行，SHA256
`A33CDDC041C292E4B6FEAFD41E5CFBA8E93493E187760A5A0072F49D938502C0`，结束标记为
`READ_ONLY_PC2_SDK2_ABI_AUDIT_OK`。PC2 工具链、x86-64 archive、ChannelFactory 符号、
4 类 HG 状态头和 DDS 共享库基础兼容性通过。

但 PC2 缺少 `unitree/idl/hg/SportModeState_.hpp` 和 H2 Loco 头文件；PC2 archive 与本地
固定 SDK archive 也不同。因此完整 H2 HAL 控制 ABI 未通过，禁止混入本地头/库。

## 9. 阶段 05（PC2 native 只读订阅）

原草案因 SportModeState 头缺失无法在 PC2 编译，现已删除该类型，并把 runner 加固为
完整哈希、全新构建目录、`ldd`、`nm`、危险字符串、7-topic 白名单、timeout 和残留进程
硬门槛：

```text
05_pc2_build_run_hg_state_read_only.sh
SHA256=4D47D53CA1B101FBB671686D130061F92AA6250E0A15BB8991E47A5FE0465808

05_hg_state_probe/CMakeLists.txt
SHA256=0D9D32404AD833ACC25EBA899AD345899F599CA2EE710A22E0014EA1DD84D492

05_hg_state_probe/h2_hg_state_read_only_probe.cpp
SHA256=D794406DFBEB4C88C1256BF24DCABA6AE3C5C9B504B10313DE0DE90A097BE844
```

隔离 Docker 复核得到 `STAGE05_HG_STATE_PROBE_OFFLINE_SAFETY_OK`；二进制未发现发送或
控制符号，嵌入名称恰为 7 个状态订阅 topic。应用不创建 DataWriter、发送通道或控制
Client；SDK2 Init 内部会预建 Publisher 容器，不能表述为“完全没有 Publisher 实体”。

随后已把这三个文件上传到 `/home/unitree/p2_unitreeH2/` 并在 PC2 本机编译、执行。
这个阶段始终不授权 `GetFsmId()`、H2 HAL、速度、动作或任何命令 topic。

### 9.1 实际 r1/r2 结果

- r1：原生编译成功，`ldd` 将 DDS 库解析到
  `/opt/unitree_robotics/lib/x86_64/libddsc*.so.0`。当时路径允许式过窄，因此在探针
  启动前输出 `BINARY_DDS_LIBRARY_PATH_MISMATCH`、`STAGE05_RUN_RC=14`。这不是 ABI 或订阅失败。
- r2：该架构子目录的动态库哈希被纳入门禁；七 topic 接收二进制审计通过，
  15 秒订阅录得 `rt/lowstate` 15,718 个样本（1,048.231 Hz）、
  `rt/secondary_imu` 15,720 个样本（1,048.254 Hz），四个 `rt/lf/*` topic
  各 302 个样本（20.158 Hz）；`rt/lowstate_raw` 为 0 样本。
- `rt/lowstate` 和 `rt/lf/lowstate` 的 CRC 分别 15,718/15,718 和 302/302 通过，没有
  CRC 失败。运行后没有探针进程残留。
- r2 最终标记：`H2_HG_SUBSCRIBE_ONLY_PROBE_OK`、
  `READ_ONLY_PC2_HG_STATE_PROBE_OK`、`STAGE05_RUN_RC=0`。

结果只解锁 H2 传感器输入适配开发，不证明 H2 HAL 已可实机控制。最终调用链、
SDK2 内部 DDS 与上层 ROS 2 解耦边界、Docker 门禁见
`docs/Unitree_H2_直接HAL控制与Docker解耦架构记录_20260716.md`。

Stage 06A 后续已在本地断网镜像中通过；它只验证 Factory、HAL 直接映射、错误边界和
自包含动态库，不包含 H2 实机 RPC。详见
`docs/Unitree_H2_HAL直接接口与Stage06A离线验收记录_20260716.md`。
