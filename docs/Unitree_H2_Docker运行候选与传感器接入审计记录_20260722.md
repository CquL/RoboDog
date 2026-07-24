# Unitree H2 Docker 运行候选与传感器接入审计记录

日期：2026-07-22

## 本轮目标

1. 完整核对现有 H2 Docker、PC2 原生运行包、传感器发现日志和官方资料。
2. 判断容器能否通过 `RobotHardwareInterface` 控制 H2。
3. 判断现有容器是否已经能接收 IMU、雷达和导航输入。
4. 构建一个不虚构雷达型号的 H2 runtime candidate。
5. 记录离线证据、实机未验证边界和下一步命令。

## 审计过的主要证据

- `unitreeH2/docker/Dockerfile`、`compose.h2.yaml`、Docker README 和 Stage 06A 验收脚本；
- `unitreeH2/README.md`、`H2_INPUT_AND_BRIDGE_CONTRACT.md`；
- Stage 05 HG 探针源码、构建脚本和 r2 实机日志；
- PC2 inventory、DDS/SDK、ROS 2 graph、topic/QoS contract 日志；
- 官方 H2 产品页、本地 H2 架构/DDS 文档、SDK2 H2 示例与 HG IDL；
- 当前 `robot_hardware` Factory、H2 HAL、统一测试程序及 safe/live 配置；
- 本机 Docker 中旧 `amd64-offline`、`amd64-live-test-candidate` 和 Jezetek 基础镜像。

## 原有 Docker 的实际状态

旧镜像 `unitree_h2:amd64-offline` 的实际 ID 为：

```text
sha256:1fea743bc70905b21a24f04c061af41e4f2513e4cda996ae5a77adf536ef3caa
```

它只有 SDK2、HAL、配置和离线合同测试；没有状态订阅器、传感器驱动、导航算法或
`h2_runtime`。`unitree_h2:amd64-live-test-candidate` 也没有传感器程序。后续 r11/r12
控制成果主要在 PC2 native bundle 中，不会自动更新上述 Docker 镜像。

## 控制链结论

容器与原厂 PC2 用户空间采用同一控制链：

```text
上层 C++ 控制器
-> RobotFactory::RobotAllocate()
-> shared_ptr<RobotHardwareInterface>
-> initRobotHardware()
-> writeRobotVelocityCommand()/writeActionCommand()
-> UnitreeH2
-> SDK2 LocoClient
-> CycloneDDS
-> PC1 sport 服务
```

HAL 不依赖 ROS 2 控制 topic。只要运行环境是兼容的 Linux amd64、容器使用 host
network、`eth0` 能访问 PC1 且 SDK2 ABI/动态库正确，同一套上层调用可以在原厂 PC2
用户空间或容器中工作。若算法和 HAL 在不同进程，C++ 虚函数不能跨进程直接调用，必须
增加项目自有 IPC；最终单进程 `h2_runtime` 尚未实现。

## IMU 与机身状态结论

H2 本体至少有两条已验证 IMU 路径：

- `rt/lowstate` 内的 pelvis IMU；
- `rt/secondary_imu` 的 torso/body IMU。

Stage 05 r2 在 PC2 原生环境中 15 秒收到：

```text
rt/lowstate       about 1048.231 Hz
rt/secondary_imu  about 1048.254 Hz
rt/lf/*           about 20.158 Hz
```

这些数据有 quaternion、gyro、acceleration、RPY 和温度；`LowState` CRC 通过。因此
H2 本体 IMU 不缺硬件驱动，容器侧需要的是 SDK2/HG DDS 订阅器。本轮已把同一只读探针
编入新镜像，但尚未完成 PC2 Docker 内实机订阅。

PC2 的 `/dog_imu_raw` 虽约 500 Hz，但样本 gyro/acceleration 为零，只能作为旧主机
候选话题，不能替代原生 HG IMU 合同。

## 雷达结论

H2 官方产品规格目前只明确大视场角仿人双目相机，没有公布 H2 标配雷达型号。H2 架构
文档提到通用“雷达点云/DDS”数据流，只能证明平台能力，不能证明本台交付机的物料。

PC2 中存在 RoboSense、Hesai、Livox 软件目录，并曾运行
`rslidar32_lidar.sh Go2`/`rs_lidar_cheak`；但审计同时发现：

- 脚本参数是 `Go2`，不是 H2 硬件确认；
- 候选地址 `192.168.124.20` 当时邻居状态不可达；
- `rs_lidar_cheak`/`h1_client` 出现类型转换异常；
- `/point_in_map` 和 `/tf` 有 endpoint，但 10 秒内无样本；
- 没有记录到稳定原始雷达点云输入。

因此当前没有证据确认物理雷达品牌、型号、IP、UDP 端口、标定或可用驱动。宇树公开有
L1/L2 LiDAR SDK 和 ROS 2 驱动，但在确认本机确实安装相应型号前不能装入 H2 镜像。

## 新 runtime candidate

新增文件：

- `unitreeH2/docker/Dockerfile.runtime`
- `unitreeH2/docker/h2_entrypoint.sh`
- `unitreeH2/docker/compose.h2.runtime.yaml`
- `unitreeH2/docker/build_h2_runtime_amd64.ps1`
- `unitreeH2/docker/verify_h2_runtime_image_offline.ps1`
- `unitreeH2/docker/verify_h2_runtime_inside.sh`
- `unitreeH2/docker/config/unitree_h2_container_safe.yaml`
- `unitreeH2/docker/config/unitree_h2_container_motion.yaml`

构建结果：

```text
image: unitree_h2:amd64-runtime-candidate
id: sha256:91649a9c5e4cde9bd4cfada646dee5bc99e10e26e76f08d8f1c999de40ee51d1
architecture: amd64
size: 3,534,141,236 bytes
scope: hal-native-hg-state-candidate
```

离线验证通过：

- `robot_hardware` CTest 6/6；
- Factory/direct API/live-motion-plan contracts；
- 所有关键 ELF `ldd` 无 `not found`；
- 只保留统一 `robot_test_unitree_h2`；
- HG state probe 参数边界；
- motion 配置缺少 `--execute`、超限值和 locked 配置非零速度拒绝门禁；
- ROS 2 Humble CLI 在镜像内可用。

成功标记：

```text
H2_RUNTIME_IMAGE_OFFLINE_OK
H2_RUNTIME_IMAGE_HOST_GATE_OK
H2_RUNTIME_IMAGE_BUILD_OK
```

传输包：

```text
unitreeH2/runtime_bundle/unitree_h2_amd64_runtime_candidate_20260722.tar.gz
size: 1,233,391,298 bytes
SHA256: defc9b86f6b65b2752326e1a51a7ad301d722e7790378845528f4d575f968f3a
```

归档 `manifest.json` 已复核，RepoTag 为
`unitree_h2:amd64-runtime-candidate`。

## 并行配置修改发现

本轮构建期间，工作区 `robot_hardware/config/unitree_h2.yaml` 被改为
`allow_motion_commands=true`、`allow_state_changing_actions=true`、`max_vx=0.70`；live
配置的 state-changing action 也被打开。`0.70` 超出适配器当前 `0.50` 项目上限，会让
初始化返回 `1001`。本轮没有覆盖这些用户修改，而是为镜像新增独立的 locked/motion
配置；motion 配置仍把状态切换锁为 false。

## 当前边界与下一步

1. 新镜像已完成本地离线构建，不等于 PC2 Docker 实机验证。
2. 最后一次 PC2 审计没有 Docker；安装 Docker Engine 属于单独系统变更。
3. PC2 首次容器测试只运行 15 秒 HG 只读订阅，验证 eth0、DDS 与镜像 ABI。
4. 只读成功后再在容器中执行 getter，随后单独审批 zero-stop 和短脉冲 HAL。
5. 雷达接入前必须实机确认传感器铭牌/BOM、IP、端口、供电、驱动和标定。
6. 下一开发项是生产型 `unitree_h2_state_source`、项目统一 IMU/Joint/Status 合同和正式
   `h2_runtime`；导航点云与相机桥在硬件合同确认后加入。
