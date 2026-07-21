# Unitree H2 EDU 输入与桥接合同（阶段 1）

更新时间：2026-07-16

## 结论

H2 应新增独立的 `unitree_h2_sensor_bridge`，不要把
`x30_livox_ros2_transfer` 改名后直接复用。桥接包只负责把 H2 的 SDK2/DDS 数据转成
项目统一传感器数据合同；如果现有算法内部使用 ROS 2，可由该适配层发布项目自有的
标准输入话题。导航算法和 `RobotHardwareInterface` 不直接依赖宇树 IDL。

控制出口不经过宇树 ROS 2 控制 topic：算法最终产生
`RobotVelocityCommand`/项目动作常量，然后直接调用 `RobotHardwareInterface`。H2 HAL 内部调用
宇树 SDK2 `LocoClient`。SDK2 内部 DDS 是厂商传输实现，不是上层 ROS 2 控制耦合。

本文件先固定接口合同。实际 ROS 2 包要等交付的 H2 EDU 硬件配置、相机接口和坐标系
资料核对后实现，避免虚构不存在的雷达或话题。

## 2026-07-16 PC2 实机发现增量

- H2/HG 的 LowState、secondary IMU、BMS、MainBoard 和 SportModeState publisher 已在
  CycloneDDS Domain 0 图中发现，publisher 均为 Reliable、KeepLast 1、Volatile。
- PC2 ROS 2 工作空间没有 `unitree_hg` 接口包，但 `/opt/unitree_robotics/include` 有
  HG C++ IDL；H2 本体输入优先使用 SDK2 native 纯 subscriber 解码，再桥接成标准消息。
- `/dog_imu_raw` 和 `/dog_odom` 由 `dog_control_pub` 以约 500 Hz 发布。前者只有姿态
  有效而 gyro/accel 为 0，不能作为完整原始 IMU；后者 covariance 全 0，只作待核对
  里程计候选。
- `/point_in_map` 和 `/tf` 虽有 publisher，但 10 秒内无样本；相机 topic 也无
  publisher。因此雷达、地图点云、TF、相机尚未纳入可用输入合同。
- `/lf/emergency_stop` 只有 subscriber，没有可读状态 publisher，不能作为软件安全
  联锁。
- PC2 SDK2/工具链 ABI 审计已完成。`LowState_.hpp`、`IMUState_.hpp`、
  `BmsState_.hpp`、`MainBoardState_.hpp` 可使用，但 HG `SportModeState_.hpp` 缺失；
  阶段 05 先做上述四类数据的 7-topic native 纯订阅，不从本地 SDK 补头混编。
- PC2 未发现 H2 Loco 头文件且静态库与本地固定 SDK 不同。因此此输入探针通过也只会
  解锁传感器桥开发，不代表 `RobotHardwareInterface` 的实机运动控制已通过。
- Stage 05 r1 已在运行前因路径门禁退出；它的 `STAGE05_RUN_RC=14` 不是订阅失败。
- Stage 05 r2 已完成 15 秒 native 纯订阅：`rt/lowstate` 和 `rt/secondary_imu`
  约 1,048 Hz，四个 `rt/lf/*` 低频状态约 20.158 Hz，`rt/lowstate_raw` 无样本；
  结束标记为 `READ_ONLY_PC2_HG_STATE_PROBE_OK`、`STAGE05_RUN_RC=0`。

完整证据见 `docs/Unitree_H2_PC2_话题合同与SDK2_ABI门槛记录_20260716.md` 和
`docs/Unitree_H2_直接HAL控制与Docker解耦架构记录_20260716.md`。

## 已由当前 H2 官方源码确认的数据

### HAL 与安全闭环

| 原厂输入 | 用途 | 统一输出建议 |
|---|---|---|
| `rt/lowstate` | mode、1 ms tick、pelvis IMU、`MotorState[35]`、遥控器原始 40 字节、reserve、CRC | `sensor_msgs/JointState`、pelvis `sensor_msgs/Imu`、`RobotStatus` |
| `rt/secondary_imu` | H2 官方示例所称 torso IMU | torso `sensor_msgs/Imu` |
| H2 Loco RPC 的 FSM/返回码 | 高层运动服务是否就绪、命令是否成功 | `RobotStatus`、诊断与命令确认 |
| 消息时间戳与本地接收时间 | 新鲜度、丢包、超时停止 | watchdog/diagnostics |

当前官方 H2 低层示例还公开 `rt/lowcmd`，但它是写入全身电机的低层控制通道，不是
算法输入。本阶段禁止发布该 topic。

## 实机必须发现、不能由其他机型推断的数据

下面是算法可能需要的候选输入，但当前 H2 一手资料尚不能保证它们在这台交付机上的
topic 名、类型和可用性：

- 电池/BMS 独立 topic；
- 原厂 odometry、pose、twist 及其坐标定义；
- 双目相机图像、编码、`CameraInfo`、内外参和硬件时间戳；
- 雷达/点云及标定。H2 产品页没有确认 Livox/MID360 为标配；
- 独立故障、主板、急停或安全状态 topic。

`rt/lf/bmsstate`、`rt/odommodestate` 等名字可作为实机搜索候选，但目前不能写成 H2
保证接口；必须用本机 DDS/ROS 2 发现结果和供应商交付文档确认。

### 感知算法

- 双目图像、`CameraInfo`、内外参、硬件时间戳和 TF：以 H2 EDU 实际交付接口为准。
- 点云：只有实机确实配置雷达并拿到驱动/标定时才纳入合同；公开产品信息不能证明
  MID360 是 H2 标配。
- 如果导航算法内部使用 ROS 2，只订阅项目统一的 `Image`、`PointCloud2`、
  `Imu`、`Odometry` 和 TF，不读取厂商私有消息。这些是算法输入，不是机器人
  控制出口。

## 不应关闭的原厂输入/服务

- 高层 `LocoClient` 依赖原厂运动控制服务，本阶段不得关闭。
- 原厂状态、遥控器和急停链路继续保留；软件 `StopMove()` 不是物理急停。
- 不照搬 X30 的 ROS1 Livox/Yesense 停启脚本。
- 只有某个物理设备或 UDP 端口被两个驱动独占且已通过抓包/进程证据确认冲突时，才
  停止对应的单个驱动。
- `rt/lowcmd` 是低层关节控制通道，需要按官方调试模式解决内置控制器冲突；当前
  H2 HAL 使用高层 RPC，禁止发布 `rt/lowcmd`。

## 容器边界

```text
H2 原厂 DDS/相机接口
  -> unitree_h2_sensor_bridge（机器人输入适配层）
  -> 稳定的项目传感器数据合同
  -> 通用算法
  -> 统一 RobotVelocityCommand / 动作
  -> RobotHardwareInterface（进程内直接调用）
  -> UnitreeH2 HAL -> H2 高层 LocoClient -> SDK2 内部 DDS -> PC1 sport
```

算法容器/镜像保持可迁移；换机器人时替换机器人输入适配/HAL，上层数据和控制
合同不变。如果要直接调用 C++ HAL 对象，最终控制器和 HAL 必须在同一进程。DDS 实机
运行需要 host network 和明确绑定 H2 有线网卡；这个不可消除的网络合同不等于依赖
PC2 ROS 2 控制环境。

## 实现前验收条件

1. 确认采购型号为 H2 EDU。
2. 确认 PC2/PC3/PC4 实际硬件、架构、OS、固件和 SDK2 版本。
3. 获取相机/雷达清单、驱动、标定、坐标系和时间同步说明。
4. Stage 05 r2 已在不发控制指令的情况下完成七个 DDS 候选 channel 的
   15 秒 native 订阅；后续仍需补相机、雷达、标定和时间同步。
5. 固定项目输入合同；若算法使用 ROS 2，再固定项目自有 topic/TF 命名和 QoS，并录制
   离线回放数据。
6. 桥接包通过断网、丢帧、过期时间戳和重启测试后，才接通算法闭环。
7. 控制链另行逐门禁验收：Docker 离线自包含 Stage 06A 已通过；下一步是镜像内生产型
   状态源纯订阅，然后才是首次只读 RPC、零速度/停止和受限非零运动。Stage 05 r2 与
   Stage 06A 都不解锁任何实机控制。

## PC2 远端工作目录

后续 H2 文件统一放在 `/home/unitree/p2_unitreeH2/`，并按 `scripts/`、`logs/`、
`src/`、`build/`、`config/` 分类。不要继续把项目文件直接放到 `/home/unitree/`，也
不要复用或修改原厂 `graph_pid_ws` 和 `slam_config`。
