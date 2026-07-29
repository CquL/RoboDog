# Unitree H2 ROS 2 IMU 桥接与镜像验收记录

日期：2026-07-28

## 计划

1. 复用已通过 H2 实机验收的 SDK2/HG DDS 订阅路径。
2. 新建独立 `unitree_h2_sensor_bridge` ROS 2 Humble 节点。
3. 将 `rt/lowstate.imu_state` 和 `rt/secondary_imu` 转成项目自有
   `sensor_msgs/msg/Imu` 话题。
4. 将节点、参数、合同测试和启动配置编译进 H2 runtime candidate。
5. 生成保持固定名称的离线迁移镜像包。

## 实际实现

新增目录：

```text
unitreeH2/sensor_bridge/
```

实现的数据合同：

| 原厂 DDS 输入 | ROS 2 输出 | frame_id |
|---|---|---|
| `rt/lowstate.imu_state` | `/h2/imu/pelvis` | `h2_pelvis_imu` |
| `rt/secondary_imu` | `/h2/imu/torso` | `h2_torso_imu` |

转换内容：

1. 宇树 `Qw,Qx,Qy,Qz` 映射为 ROS orientation 的 `w,x,y,z`。
2. gyroscope 映射为 `angular_velocity`。
3. accelerometer 映射为 `linear_acceleration`。
4. 拒绝 NaN、Inf 和无效零范数四元数，发布前归一化四元数。
5. 原厂 `IMUState_` 没有完整 ROS 源时间戳，首版使用容器接收时间。
6. 三组 covariance 保持全零，语义为“数据存在但协方差未知”；不错误使用
   `-1` 把有效测量标记为不存在。
7. 默认从约 1 kHz 原厂 DDS 输入限频发布为 200 Hz，发布频率可配置。
8. 100 ms 无新样本时停止发布过期数据并输出节流警告。

## DDS 与 ROS 2 隔离

```text
H2_DDS_DOMAIN=0
ROS_DOMAIN_ID=20
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

H2 SDK2 在进程内部继续使用其自带 CycloneDDS 连接原厂 Domain 0；项目 ROS 2
使用基础镜像已有的 Fast DDS RMW 和 Domain 20。节点不订阅 PC2 原厂 ROS 2
话题，也不发布任何 H2 控制通道。

## 构建问题与修正

1. Docker `RUN set -u` 与 `/opt/ros/humble/setup.sh` 的未定义变量检查冲突；
   在 source ROS 环境时局部关闭 `nounset`，source 后恢复。
2. CMake 命令行的 `CMAKE_PREFIX_PATH` 必须使用分号列表，不能把冒号字符串作为
   单个 CMake 路径。
3. ament Python 模块依赖 ROS setup 注入的 `PYTHONPATH`，因此不能只传
   `CMAKE_PREFIX_PATH`。
4. 基础镜像不存在 `rmw_cyclonedds_cpp`，ROS 2 端改用已安装的
   `rmw_fastrtps_cpp`；SDK2 的 CycloneDDS 不受影响。
5. ROS 2 Humble 的 `rclcpp::Time` 接口与新版本不同，改为显式转换
   `builtin_interfaces::msg::Time`。

## 离线验收结果

```text
UNITREE_H2_IMU_MAPPING_CONTRACT_OK
H2_IMU_BRIDGE_READY
/h2/imu/pelvis [sensor_msgs/msg/Imu]
/h2/imu/torso [sensor_msgs/msg/Imu]
H2_IMU_BRIDGE_OFFLINE_ENDPOINTS_OK
H2_RUNTIME_IMAGE_OFFLINE_OK
H2_RUNTIME_IMAGE_HOST_GATE_OK
H2_RUNTIME_IMAGE_BUILD_OK
```

镜像：

```text
tag: unitree_h2:amd64-runtime-candidate
image id: sha256:3e704af1b403ace050ca514ca3238d8d9b52d2774faf28286e0a59d8349fa692
architecture: amd64
scope: hal-native-hg-state-ros2-imu-candidate
```

固定名称迁移包：

```text
unitreeH2/runtime_bundle/unitree_h2_amd64_runtime_candidate.tar.gz
SHA256 dc878fce719591edbe671989662e7d30fa75e38543ef8fd5d646d6f7e1d943be
```

## 迁移边界

迁移到另一台兼容 H2 时，不需要复制源码和重新编译；复制上述镜像包和 SHA256，
在目标 PC2 安装 Docker 后 `docker load` 即可。每台机器人仍必须核对或提供：

1. H2 EDU/固件/SDK2 服务兼容性。
2. 实际连接 PC1 的网卡名，默认是 `eth0`。
3. 原厂 DDS Domain，当前交付机为 `0`。
4. 项目 ROS Domain，默认使用 `20`。
5. 两个 IMU 的安装方向、frame、外参、单位和标定协方差。

镜像本身包含 H2 HAL 和双 IMU 桥，但当前不包含导航算法、雷达/相机驱动、
Odometry/TF/JointState/RobotStatus 桥和正式 `h2_runtime`。因此仅迁移当前镜像可
获得 H2 DDS 接入、抽象 HAL 二进制和 ROS 2 IMU 话题能力，不能直接获得完整自主导航。

## 下一步实机门禁

1. 在 H2 PC2 加载新镜像。
2. 上传并执行
   `unitreeH2/remote/10_pc2_h2_docker_imu_bridge_read_only.sh`；脚本只启动
   `unitree_h2_sensor_bridge`，不启动控制程序。
3. 验证两个话题的类型、频率、frame_id、时间戳单调性和数据新鲜度。
4. 机器人静止时验证加速度模长约为重力加速度、角速度接近零、四元数范数约为 1。
5. 分别绕三轴缓慢转动，确认坐标轴符号和 pelvis/torso 对应关系。
6. 验收完成后再接入导航滤波器；本阶段不运行速度或动作命令。

## 第一次实机启动结果与修复

2026-07-28 第一次实机启动的实际结果：

1. `unitree_h2_amd64_runtime_candidate.tar.gz` 的 SHA256 校验通过。
2. `docker load` 成功；同名标签由旧镜像
   `sha256:f13b7a2b986662a12be9280cda41e6321c66f5acb658135a271e522b28302f8c`
   切换到新镜像，旧镜像被取消标签属于 Docker 正常行为。
3. 只读脚本使用 `eth0`、DDS Domain `0`、ROS Domain `20` 启动了容器
   `813839cc001ad7a2689d97966a6d83283661c60e3245bc5d8a3ff77c26e345cc`；
   实机日志为
   `/home/unitree/p2_unitreeH2/logs/h2_docker_imu_bridge_20260728_124921.log`。
4. 节点在输出 `H2_IMU_BRIDGE_READY` 以及接收任何 ROS 2 IMU 样本之前退出：

   ```text
   failed to configure logging:
   Failed to create log directory: /root/.ros/log
   ```

   其中 `rcl context unexpectedly not shutdown during cleanup` 是上述初始化异常后的
   清理信息，不是首要故障。

5. 根因是容器保持 `--read-only` 时，ROS 2 默认日志目录
   `/root/.ros/log` 不可创建；这是 ROS 2 日志目录运行配置问题，不是 DDS、
   网卡、ABI 或 IMU 字段转换失败。
6. 已在实机脚本和 Compose 的 IMU bridge 服务中增加
   `ROS_LOG_DIR=/tmp/ros_logs`，复用现有受限 `/tmp` tmpfs；不关闭
   `--read-only`，不增加容器权限。该修复不改变镜像内容，因此当前实机只需
   重新上传脚本，无需重新传输镜像包。

本机已在相同的只读、无额外 capability、`/tmp` tmpfs 约束下复验：
增加 `ROS_LOG_DIR=/tmp/ros_logs` 后节点能够输出 `H2_IMU_BRIDGE_READY`。
实机双 IMU 样本验收仍为待重跑；只有脚本最终输出
`H2_DOCKER_IMU_BRIDGE_LIVE_SAMPLES_OK` 后才可记录为通过。
