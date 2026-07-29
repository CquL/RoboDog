# Unitree H2 Docker IMU 使用说明

## 当前镜像

```text
镜像标签：unitree_h2:amd64-runtime-candidate
Windows Docker Desktop OCI 索引 ID：
sha256:f7cd06b3d28d90b68cffda3754050203a1cc438207ebf575cc090ad40b6e7d3c
H2 PC2 Docker Engine 24 amd64 镜像配置 ID：
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
镜像归档：unitree_h2_amd64_runtime_candidate.tar.gz
SHA256：ba8025b3250d01544a41272dc50b8b71803d86f582856915ee6f8d7009a1f5a5
适用平台：H2 PC2，Linux/amd64，Ubuntu 22.04
```

`f7cd06...` 是 Docker Desktop/buildx 显示的 OCI 索引摘要；`f8f811...` 是 H2
Docker Engine 24 加载 amd64 镜像后显示的镜像配置 ID。两者属于同一归档的不同
OCI 对象，不能跨 Docker 实例直接用数值相等判断是否为同一版本。应在 H2 的同一个
Docker daemon 内比较容器的 `.Image` 与镜像标签的 `.Id`。

镜像内的 `unitree_h2_sensor_bridge` 直接订阅 H2 原厂 DDS 数据，并发布：

```text
rt/lowstate.imu_state -> /h2/imu/pelvis [sensor_msgs/msg/Imu]
rt/secondary_imu      -> /h2/imu/torso  [sensor_msgs/msg/Imu]
```

## 启动 Docker 与查看容器

启动 Docker Engine，并启动已经创建过但当前停止的 IMU Bridge 容器：

```bash
sudo systemctl start docker
sudo docker start unitree-h2-imu-bridge
```

`docker start` 不会创建新容器，也不会把旧容器升级到刚加载的新镜像。

只查看正在运行的容器：

```bash
sudo docker ps
```

查看全部容器，包括已经停止的容器：

```bash
sudo docker ps -a --format \
  'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}'
```

确认现有容器是否仍绑定旧镜像：

```bash
sudo docker inspect \
  --format 'container_image_id={{.Image}}' \
  unitree-h2-imu-bridge

sudo docker image inspect \
  --format 'tag_image_id={{.Id}}' \
  unitree_h2:amd64-runtime-candidate
```

两个 ID 不一致时，`docker start` 仍会运行旧镜像；必须按本文第 3 节重新创建容器。

### 2026-07-28 PC2 实际部署验收

```text
container: unitree-h2-imu-bridge
container ID: a7604d3b3ce70c3a3f72b610cee01577a953355289e1884464897fdafe4d01a9
container image ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
tag image ID:
sha256:f8f8115cdf57b6266beaadc5814a21de961e00ae585e86daa7bdb08854593451
architecture: amd64
status: Up
motion config: max_vx=1.00, max_omega=0.70
```

容器的镜像 ID 与 PC2 上标签的镜像 ID 完全一致，证明当前运行容器已经使用本次
新加载的镜像；容器内配置检查也证明新的速度包络已经写入镜像。无需再次上传或重建。

## 1. 将镜像复制到另一台 H2

在 Windows PowerShell 执行，按实际情况修改机器人 IP：

```powershell
scp `
  "D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_amd64_runtime_candidate.tar.gz" `
  "D:\Desktop\RoboDog\unitreeH2\runtime_bundle\unitree_h2_amd64_runtime_candidate.tar.gz.sha256" `
  unitree@192.168.123.162:/home/unitree/p2_unitreeH2/images/
```

## 2. 在 H2 PC2 导入镜像

```bash
cd /home/unitree/p2_unitreeH2/images

sha256sum --check --strict \
  unitree_h2_amd64_runtime_candidate.tar.gz.sha256

gzip -dc unitree_h2_amd64_runtime_candidate.tar.gz |
sudo docker load

sudo docker image inspect \
  unitree_h2:amd64-runtime-candidate \
  --format 'ID={{.Id}} ARCH={{.Architecture}}'
```

## 3. 启动 IMU Bridge

以下命令只接收状态并发布 IMU，不发送运动控制命令：

```bash
sudo docker rm -f unitree-h2-imu-bridge 2>/dev/null || true

sudo docker run -d \
  --name unitree-h2-imu-bridge \
  --network host \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  -e H2_DDS_INTERFACE=eth0 \
  -e H2_DDS_DOMAIN=0 \
  -e ROS_DOMAIN_ID=20 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/ros_logs \
  unitree_h2:amd64-runtime-candidate \
  /opt/robodog/bin/unitree_h2_sensor_bridge \
    --ros-args \
    --params-file \
    /opt/robodog/share/unitree_h2_sensor_bridge/config/unitree_h2_imu_bridge.yaml
```

确认节点就绪：

```bash
sudo docker logs unitree-h2-imu-bridge
```

应能看到：

```text
H2_IMU_BRIDGE_READY
```

## 4. 进入容器查看 IMU

```bash
sudo docker exec -it unitree-h2-imu-bridge bash

sudo docker exec -it unitree-h2-imu-bridge \
  bash --noprofile --norc
```

在容器内执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_DISABLE_DAEMON=1

ros2 topic list -t | grep '^/h2/imu'

ros2 topic echo \
  /h2/imu/pelvis \
  sensor_msgs/msg/Imu \
  --once --no-arr

ros2 topic echo \
  /h2/imu/torso \
  sensor_msgs/msg/Imu \
  --once --no-arr
```

查看发布频率：

```bash
timeout 10s ros2 topic hz /h2/imu/pelvis
timeout 10s ros2 topic hz /h2/imu/torso
```

确认当前处于容器内：

```bash
test -f /.dockerenv && echo IN_DOCKER
```

## 5. 退出和停止

```bash
exit
sudo docker rm -f unitree-h2-imu-bridge
```

## 迁移条件

- 目标必须是 Linux/amd64 的 H2 PC2；该镜像不能直接用于 ARM64 的 PC3/PC4。
- Docker 必须使用 `--network host`，否则 DDS 组播可能无法发现 H2 数据。
- 当前实机 DDS 网卡是 `eth0`。如果另一台 H2 的网卡名不同，应修改
  `H2_DDS_INTERFACE`。
- 只迁移该镜像即可运行 IMU Bridge，不需要在目标 H2 重新编译 SDK2 或 ROS 2
  节点。
