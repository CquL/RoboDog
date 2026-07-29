# Unitree H2 速度包络 1.0/0.7 与 Docker 镜像重建记录

日期：2026-07-28

## 计划

1. 核对宇树公开 H2 `SetVelocity()` 资料与数值边界。
2. 将项目 H2 前向速度硬上限改为 `1.00 m/s`，转向角速度硬上限改为
   `0.70 rad/s`。
3. 同步源码、运动 YAML、契约测试和 Docker 镜像内验收。
4. 离线重建并重新导出固定名称的 H2 runtime candidate。

## 官方资料结论

- 官方接口为 `SetVelocity(vx, vy, omega, duration)`，API ID 为 `7105`。
- 官方 RPC 文档示例为 `vx=0.5 m/s`、持续 `1 s`。
- 宇树公开 SDK、H2 开发文档和 H2 产品页没有公布双足 H2 的
  `vx/vy/omega` 合法范围或最大值。
- 因此 `1.00/0.70` 是本项目允许上限，不是宇树官方最大值，也不能视为已验证的
  实机安全范围。

## 实际修改

- `robot_hardware/robot_hardware/src/unitree/unitree_h2.cpp`
  - `kProjectMaxVx: 0.50 -> 1.00`
  - `kProjectMaxOmega: 0.30 -> 0.70`
- 同步 `unitree_h2.yaml`、`unitree_h2_live.yaml` 和
  `unitree_h2_container_motion.yaml`。
- `max_vy=0.10`、`velocity_command_duration_s=0.30` 保持不变。
- `unitree_h2_container_safe.yaml` 保持禁止运动和保守限幅。
- 增加 `1.01 m/s` 与 `0.71 rad/s` 越界拒绝契约。
- 头文件与 CLI 缺字段时的保守回退值未提高。

注意：`unitree_h2_container_motion.yaml` 中
`allow_state_changing_actions: true` 是本轮开始前已经存在的独立工作区修改；本轮没有
替用户回退，也没有通过本轮速度修改自动调用任何状态动作。

## 离线验收结果

```text
robot_hardware CTest: 7/7 passed
UNITREE_H2_FACTORY_CONTRACT_OK
UNITREE_H2_DIRECT_API_CONTRACT_OK
UNITREE_H2_LIVE_MOTION_PLAN_OK
UNITREE_H2_IMU_MAPPING_CONTRACT_OK
H2_IMU_BRIDGE_OFFLINE_ENDPOINTS_OK
H2_RUNTIME_IMAGE_OFFLINE_OK
H2_RUNTIME_IMAGE_HOST_GATE_OK
H2_RUNTIME_IMAGE_BUILD_OK
```

本轮没有连接 H2 实机，没有发送速度或动作命令。

## 新镜像与迁移包

```text
image: unitree_h2:amd64-runtime-candidate
Windows Docker Desktop OCI index ID:
sha256:f7cd06b3d28d90b68cffda3754050203a1cc438207ebf575cc090ad40b6e7d3c
H2 PC2 Docker Engine 24 amd64 image config ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
architecture: amd64
image size: 3,534,893,846 bytes

archive:
unitreeH2/runtime_bundle/unitree_h2_amd64_runtime_candidate.tar.gz
archive size: 1,234,136,106 bytes
SHA256:
ba8025b3250d01544a41272dc50b8b71803d86f582856915ee6f8d7009a1f5a5
```

`f7cd06...` 与 `f8f811...` 分别是同一归档中的 OCI 索引摘要和 amd64 镜像配置
摘要。H2 上是否运行新镜像，应在 PC2 同一 Docker daemon 内比较容器 `.Image`
与标签 `.Id`，不能直接拿 Docker Desktop 的索引摘要与 PC2 的配置摘要比较。

## PC2 实际部署验收

2026-07-28 在 H2 PC2 完成新镜像加载和 IMU Bridge 容器重建：

```text
container: unitree-h2-imu-bridge
container ID: a7604d3b3ce70c3a3f72b610cee01577a953355289e1884464897fdafe4d01a9
container image ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
tag image ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
architecture: amd64
container status: Up
container motion YAML: max_vx=1.00, max_omega=0.70
```

容器镜像 ID 与标签镜像 ID 一致，且容器内读取到新的速度包络，故新镜像部署已经
确认成功。本次记录仅验证镜像和配置，没有发送 `1.00 m/s` 或 `0.70 rad/s` 实机命令。

## 后续实机门禁

PC2 需要先校验、加载新归档并重建容器。`docker start` 只会启动旧容器，不会让旧容器
自动切换到新镜像。首次实机验证仍应从已经验证过的 `vx=0.50` 和较低角速度开始，
逐档验证，不能把项目上限直接当作首次测试值。
