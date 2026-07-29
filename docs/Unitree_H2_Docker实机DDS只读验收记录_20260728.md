# Unitree H2 Docker 实机 DDS 只读验收记录

日期：2026-07-28

## 计划

1. 在 H2 PC2 上离线安装 Docker Engine。
2. 加载 `unitree_h2:amd64-runtime-candidate`。
3. 以只读、无额外 capability、host network 的容器运行 HG DDS 状态探针。
4. 验证 `eth0`、DDS Domain 0、SDK2/HG ABI、状态样本和 `LowState` CRC。
5. 本阶段不调用速度、停止或动作接口。

## 实际执行环境

```text
host: unitree-H2-pc2
host OS: Ubuntu 22.04 amd64
Docker Engine: 24.0.7
containerd: 1.7.12
runc: 1.1.12-0ubuntu2~22.04.1
image: unitree_h2:amd64-runtime-candidate
DDS interface: eth0
DDS domain: 0
duration: 15 seconds
container network: host
container filesystem: read-only
capabilities: all dropped
no-new-privileges: enabled
```

## 实际结果

```text
rt/lowstate_raw:
  samples=0

rt/lowstate:
  samples=15708
  rate_hz=1047.659
  crc_ok=15708
  crc_fail=0

rt/lf/lowstate:
  samples=302
  rate_hz=20.146
  crc_ok=302
  crc_fail=0

rt/secondary_imu:
  samples=15710
  rate_hz=1047.619

rt/lf/secondary_imu:
  samples=302
  rate_hz=20.146

rt/lf/bmsstate:
  samples=302
  rate_hz=20.146

rt/lf/mainboardstate:
  samples=302
  rate_hz=20.146

H2_HG_SUBSCRIBE_ONLY_PROBE_OK
PROBE_RC=0
```

日志路径：

```text
/home/unitree/p2_unitreeH2/logs/h2_docker_hg_probe_20260728_103402.log
```

## 验收结论

1. H2 PC2 Docker Engine 离线安装完成且服务运行正常。
2. H2 runtime candidate 已能在容器内通过 `eth0`、DDS Domain 0 接收原厂 HG DDS。
3. pelvis IMU、secondary/body IMU、关节状态、BMS 和 MainBoard 数据均有有效样本。
4. `LowState` 15 秒内无 CRC 失败。
5. 本次容器只有 DDS 读取，没有运行速度、停止、动作或低层控制写通道。
6. `rt/lowstate_raw` 无样本不影响本次验收，因为正式 `rt/lowstate` 已稳定接收。
7. 本次 DDS 探针验收时尚未实现生产型 ROS 2
   `unitree_h2_sensor_bridge`；后续桥接实现与实机验收状态见
   `Unitree_H2_ROS2_IMU桥接与镜像验收记录_20260728.md`。导航算法闭环和正式
   `h2_runtime` 当前仍未实现。

## 抽象 HAL 只读初始化追加验收

在同一镜像内使用容器安全配置执行：

```text
/opt/robodog/bin/robot_test_unitree_h2
  --config /opt/robodog/config/unitree_h2_container_safe.yaml
  --read-only
```

实机输出：

```text
[UnitreeH2] Initialized on DDS interface eth0.
allow_motion_commands=false
allow_state_changing_actions=false
H2_READ_ONLY_INIT_OK
```

追加结论：

1. 容器内 `RobotFactory -> UnitreeH2 -> initRobotHardware()` 路径通过。
2. H2 SDK2 高层 RPC/DDS 初始化在容器内通过。
3. 安全配置保持速度和状态切换权限关闭。
4. 本次追加验收仍未调用速度、停止或动作接口。

## Getter Audit 追加验收

容器安全配置下执行 Getter Audit，实机结果：

```text
GETTER_RC=0
fsm_id=601
fsm_ret=0
fsm_mode=0
mode_ret=0
available_ret=0
available_count=264
H2_GETTER_ONLY_RPC_OK
```

结论：

1. 容器内 `GetFsmId`、`GetFsmMode` 和 `GetAvailableFsmIds` 均调用成功。
2. 本次实机状态为 `601:HybridWalk`。
3. Getter Audit 只有读取操作，没有切换 FSM，也没有发送速度或动作指令。

## 下一步

1. 实现生产型 `unitree_h2_state_source`。
2. 实现项目统一 IMU、JointState 和 RobotStatus 输出。
3. 确认实机是否安装雷达及其品牌、型号、IP、端口、驱动和标定。
4. 只在上述门禁通过后，单独安排容器内零停止和短脉冲运动测试。

## PC2 点云来源只读复核

2026-07-28 在 PC2 原厂 ROS 2 环境中复核点云相关图谱：

1. `/QtServer` 订阅下列 LIO-SAM 处理后点云：
   - `/lio_sam_ros2/deskew/cloud_deskewed`
   - `/lio_sam_ros2/mapping/cloud_registered_z_limit`
   - `/lio_sam_ros2/mapping/map_local`
2. `/QtServer` 发布 `/point_in_map`，同时发布地图、拓扑和 PCD 管理相关接口。
3. `/QtServer` 参数为 `robot_type: Go2`、`slam_version: 2.1.1.1`，
   说明该节点属于原厂通用/历史 SLAM 图形服务栈，不能据此认定为 H2 深度相机驱动。
4. 当前 ROS 2 图中没有发现活动的 `camera`、`depth` 或 `image` 图像发布话题。
5. 对 `/point_in_map` 等待 15 秒没有收到一帧 `PointCloud2`。

结论：

1. 当前没有实时点云数据可供 Docker 接入。
2. `/point_in_map` 是 QtServer 的地图/处理后点云输出，不是传感器原始点云接口。
3. 现有证据不能把历史看到的点云归因于深度相机；其上游命名更符合
   LIO-SAM 的激光雷达点云处理链，也可能来自已加载 PCD。
4. 在确认实际传感器型号和找到活动原始点云发布者以前，不将
   `/point_in_map` 作为 H2 生产雷达或深度相机输入合同。
