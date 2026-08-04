#!/usr/bin/env bash

# H2 Docker IMU Bridge 实机只读验收脚本。
# 用途：临时启动候选镜像内 unitree_h2_sensor_bridge，确认其能从 PC1 的
# rt/lowstate 与 rt/secondary_imu 接收 DDS，并在容器内发布两路 ROS 2 Imu。
# 安全边界：容器为只读根文件系统、无额外 capability，节点只订阅状态，
# 不创建 LocoClient 或控制写通道；本脚本不测试运动。

# 任一镜像、容器、日志、节点或样本检查失败即退出，并触发清理。
set -Eeuo pipefail

# 可通过环境变量选择镜像/临时容器和通信 Domain；默认值对应已验收 PC2。
image="${H2_IMAGE:-unitree_h2:amd64-runtime-candidate}"
container="${H2_IMU_CONTAINER:-unitree-h2-imu-bridge-audit}"
dds_interface="${H2_DDS_INTERFACE:-eth0}"
dds_domain="${H2_DDS_DOMAIN:-0}"
ros_domain="${H2_ROS_DOMAIN_ID:-20}"
ros_log_dir="${H2_ROS_LOG_DIR:-/tmp/ros_logs}"
# PC2 上的正式审计日志目录和本次唯一日志文件。
log_root="/home/unitree/p2_unitreeH2/logs"
stamp="$(date +%Y%m%d_%H%M%S)"
log="${log_root}/h2_docker_imu_bridge_${stamp}.log"

mkdir -p "$log_root"

# 无论成功、失败或收到信号，都停止并删除临时审计容器，避免后台残留。
cleanup() {
  docker stop --time 3 "$container" >/dev/null 2>&1 || true
  docker rm -f "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# 从此处起 stdout/stderr 同时进入终端和持久日志。
exec > >(tee "$log") 2>&1

# 把实际镜像、网卡、DDS/ROS Domain 和日志路径写入审计头。
echo "H2_DOCKER_IMU_AUDIT_MODE=READ_ONLY"
echo "IMAGE=$image"
echo "DDS_INTERFACE=$dds_interface"
echo "DDS_DOMAIN=$dds_domain"
echo "ROS_DOMAIN_ID=$ros_domain"
echo "ROS_LOG_DIR=$ros_log_dir"
echo "LOG=$log"

# 启动前确认 Docker daemon 可用且候选镜像存在。
docker version >/dev/null
docker image inspect "$image" >/dev/null

# 镜像 scope 标签必须精确匹配 H2 HAL + HG 状态 + ROS 2 IMU 候选范围，
# 防止误用 X30 或不含只读桥接节点的镜像。
scope="$(
  docker image inspect "$image" \
    --format '{{index .Config.Labels "io.robodog.h2.runtime.scope"}}'
)"
test "$scope" = "hal-native-hg-state-ros2-imu-candidate"

# 拒绝覆盖同名容器，避免删除运维人员正在使用的实例。
if docker container inspect "$container" >/dev/null 2>&1; then
  echo "Container already exists: $container" >&2
  exit 20
fi

# 使用 host 网络让 SDK2 DDS 直接绑定 eth0；安全选项限制文件系统和权限。
# 容器主进程就是 bridge 节点，参数文件提供 DDS->ROS 话题映射。
docker run -d \
  --name "$container" \
  --network host \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  -e "H2_DDS_INTERFACE=$dds_interface" \
  -e "H2_DDS_DOMAIN=$dds_domain" \
  -e "ROS_DOMAIN_ID=$ros_domain" \
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  -e "ROS_LOG_DIR=$ros_log_dir" \
  "$image" \
  /opt/robodog/bin/unitree_h2_sensor_bridge \
    --ros-args \
    --params-file \
    /opt/robodog/share/unitree_h2_sensor_bridge/config/unitree_h2_imu_bridge.yaml \
    -p "dds_interface:=$dds_interface" \
    -p "dds_domain:=$dds_domain"

# 给 DDS 发现和首帧缓存预留时间，然后验证节点 ready 日志。
sleep 4
docker logs "$container"
docker logs "$container" 2>&1 | grep -F "H2_IMU_BRIDGE_READY"

# 在同一容器/ROS Domain 内列话题并各读取一帧 pelvis/torso Imu；
# timeout 保证断流时有界退出，echo 只订阅不发布。
docker exec "$container" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS2CLI_DISABLE_DAEMON=1
  ros2 topic list -t
  timeout 15s ros2 topic echo \
    /h2/imu/pelvis sensor_msgs/msg/Imu --once --no-arr
  timeout 15s ros2 topic echo \
    /h2/imu/torso sensor_msgs/msg/Imu --once --no-arr
'

# 只有两路实时样本均收到后才输出稳定成功标记。
echo "H2_DOCKER_IMU_BRIDGE_LIVE_SAMPLES_OK"
