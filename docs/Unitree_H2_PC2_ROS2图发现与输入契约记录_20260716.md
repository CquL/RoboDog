# Unitree H2 PC2 ROS 2 图发现与输入契约记录

日期：2026-07-16  
输入：`unitreeH2/remote/h2_pc2_ros2_graph_20260716.log`  
状态：SDK2/工具链 ABI 审计完成；等待 PC2 native HG 只读订阅  
安全边界：本轮没有发布 topic、调用运动服务、发送 action goal、运行 H2 HAL、发送
零/非零速度、执行 `StopMove()`、切换 FSM 或写入 `/lowcmd`、`/arm_sdk`。

## 1. 日志完整性

```text
size=30270 bytes
SHA256=8403B8FD3403A1CC71ED2CE41F35F33323871A6F5F5E26AF7D1B4BB770B0A1D9
end marker=READ_ONLY_PC2_ROS2_GRAPH_AUDIT_OK
```

采集环境为 ROS 2 Humble、`rmw_cyclonedds_cpp`、Domain 0、非 localhost，并使用
`/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml`。脚本到达结束标记；由于
旧版本使用 `set +e` 且没有逐条记录返回码，结束标记不等于每个子命令均为 `rc=0`。

## 2. 本轮已经证明的事实

### 2.1 H2 高层运动 RPC 候选存在

- `/api/sport/request`：4 个 publisher、1 个 subscriber；
- `/api/sport/response`：1 个 publisher、4 个 subscriber。

当前固定版本 H2 SDK2 在 `h2_loco_api.hpp` 中定义
`LOCO_SERVICE_NAME = "sport"`，所以 `/api/sport` 与 H2 `LocoClient` 使用的服务名
一致。该拓扑符合“服务端双向端点存在、多个客户端端点存在”的形态。

这仍不能证明：

- 服务端一定来自 PC1 `192.168.123.161`；
- `GetFsmId()` API 7001 能成功往返；
- 当前 HAL 已经具备实机运动条件。

`/api/loco` 只发现客户端方向的半边端点，不是当前 H2 HAL 的目标服务。
`/api/motion_switcher` 双向端点完整，但它会改变/释放运动服务，本阶段禁止调用。

### 2.2 H2/HG 状态类型已出现在 DDS 图中

| 候选输入 | 类型 | Pub/Sub | 当前判断 |
|---|---|---:|---|
| `/lowstate_raw` | `unitree_hg/msg/LowState` | 1/0 | 最明确的 H2/HG 低状态候选 |
| `/lowstate` | `unitree_go` 和 `unitree_hg` `LowState` | 1/4 | 双类型聚合，来源仍有歧义 |
| `/lf/lowstate` | `unitree_go` 和 `unitree_hg` `LowState` | 1/3 | 双类型聚合，来源仍有歧义 |
| `/secondary_imu` | `unitree_hg/msg/IMUState` | 1/1 | H2/HG IMU 候选 |
| `/lf/secondary_imu` | `unitree_hg/msg/IMUState` | 1/1 | H2/HG IMU 候选 |
| `/lf/bmsstate` | `unitree_hg/msg/BmsState` | 1/4 | BMS 候选，尚未读取样本 |

图中的 ROS 类型名只证明远端端点声明了这些类型。PC2 当前 sourced 环境的包和接口
清单中有 `unitree_api`、`unitree_go`，但没有 `unitree_hg`；因此 ROS 2 CLI 当前未被
证明能够反序列化上述 H2 消息。后续输入桥可以选择 SDK2 的 HG DDS 类型，但必须先
单独验证 IDL/ABI 和实际样本，不能把“发现类型名”当成“数据接入完成”。

### 2.3 标准 ROS 2 数据候选

| 候选输入 | 类型 | Pub/Sub | 限制 |
|---|---|---:|---|
| `/dog_imu_raw` | `sensor_msgs/msg/Imu` | 1/0 | 名称来自复用软件，未证明是 H2 权威 IMU |
| `/dog_odom` | `nav_msgs/msg/Odometry` | 1/0 | 未证明来源、频率、frame 和有效性 |
| `/point_in_map` | `sensor_msgs/msg/PointCloud2` | 1/0 | 未证明来源雷达型号或持续发布 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 1/0 | 未读取 frame tree；图中没有 `/tf_static` |

Unitree SLAM、LIO-SAM 的点云和里程计候选大多只有 subscriber，没有 publisher，不能
宣称 SLAM/LIO 正在输出。Hesai、Livox、RSLidar 包存在也不能证明物理雷达在线。

当前图中没有 `sensor_msgs/Image`、`CompressedImage` 或 `CameraInfo` publisher。
`/frontvideostream` 只有 subscriber；`/pctoimage_local` 是自定义点云投影类型，不能
当成相机图像。因此 H2 相机接入仍是未完成项。

## 3. 不能触碰的输出端点

- `/lowcmd`：`unitree_hg/msg/LowCmd`，存在活动 publisher/subscriber；
- `/arm_sdk`：`unitree_hg/msg/LowCmd`，存在活动 publisher/subscriber；
- `/api/motion_switcher`：存在完整请求/响应端点；
- `/lf/emergency_stop`：本快照只有 subscriber，没有读取到可作为安全联锁的状态。

发现端点不构成写入授权。当前仍禁止向这些端点发布或调用，也不能把软件
`StopMove()` 当成硬件急停。

## 4. 与 `RobotHardwareInterface` 的关系

当前代码映射方向仍然正确：

```text
initRobotHardware()
  -> ChannelFactory::Init(domain, NIC)
  -> H2 LocoClient::Init()
  -> GetFsmId()                         只读握手，尚未实机执行

writeRobotVelocityCommand(cmd)
  -> H2 LocoClient::SetVelocity(...)    当前安全配置默认拒绝

writeActionCommand(action)
  -> StopMove/StandUp/Start/Damp/...    当前安全配置默认拒绝状态动作
```

本轮图发现把 `sport` 服务名与实机 DDS 图对上了，但没有执行 `GetFsmId()`，所以只能说
“高层协议候选匹配”，不能说“H2 已可控制”。`unitree_hg` ROS 消息包缺失不阻断
SDK2 的高层 `LocoClient` 设计，但会影响 ROS CLI 和未来 ROS 2 传感器桥的直接解码。

## 5. ROS 2 CLI daemon 偏差与修正

采集前没有 ROS 2 CLI daemon；采集后出现 PID 70289 的
`ros2cli.daemon.daemonize`。原因是已上传执行的 `02` 版本调用了 Humble 中不支持
`--no-daemon` 的 `ros2 action list`。这个进程只是 ROS 2 CLI 图缓存进程，不是宇树
运动服务，也没有发送机器人控制，但它违反了“不留下持久 daemon”的审计目标。

版本追溯：

```text
02 已上传并执行版本 SHA256=
78712811BAD0318CEB66F0AFB06D8D7B6F6CB0504B1386C972ECC9D19FBCA935

02 当前本地修正版 SHA256=
339F2165C8DE93BBF4FB901F7C6B6A27323894C84F1252D31E6529FEAE1BA5A6
```

本地修正版已省略 action 枚举。下一脚本会先只停止该 ROS 2 CLI daemon，结束时再检查
一次；不会停止 `/QtServer`、`bms_ctr_node`、`dog_control_pub`、`switch_node` 或任何
原厂 systemd 服务。

## 6. 下一步：选定话题合同与活性审计

脚本：

```text
unitreeH2/remote/03_pc2_topic_contract_read_only_audit.sh
SHA256=CC4E28BA5B6868103BCBD14C6F00D0FB54CBB82D6C8017323C9909F0BA64FD7A
```

它将：

1. 清理由 `02` 审计产生的 ROS 2 CLI daemon；
2. 检查 `unitree_api`、`unitree_hg` 的本地包和接口支持；
3. 只读搜索 H2/HG IDL、H2 Loco 头文件和 SDK2/DDS 库，并记录真实路径与 SHA256；
4. 对选定 topic 执行 verbose endpoint/QoS 查询；
5. 对 `/dog_imu_raw`、`/dog_odom`、`/point_in_map`、`/tf` 做限时接收频率检查；
6. 只读取各一条标准 IMU 和 odometry 样本；
7. 为每个检查打印 `COMMAND_RC[...]`，结束时再次确认 CLI daemon 已退出。

它不发布 topic、不调用 service、不发送 action goal、不执行 SDK 控制 RPC。`topic hz`
由 `timeout` 限定 10 秒，观察窗口结束出现 `COMMAND_RC=124` 是预期超时，不表示脚本
失控。

`03` 已执行完成。结果表明标准 IMU/odom 有数据但质量合同未通过，点云/TF 无活性，
且 PC2 SDK archive 与本地快照不同。阶段 04 已完成；下一步只运行去除缺失
SportModeState 后的 native HG 纯 subscriber。当前仍不调用 `GetFsmId()`、不运行
`--zero-stop`、不测试任何运动。详见
`docs/Unitree_H2_PC2_远端迁移与SDK2_ABI审计记录_20260716.md`。
