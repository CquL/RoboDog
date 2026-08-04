#!/bin/bash
set -euo pipefail

# 106 生产启动入口。使用 host 网络运行 TCP-to-ROS2 接收端，
# 不挂载传感器设备、不停止原厂进程，也不发送机器人命令。
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
readonly NAME="x30_ros2_passive"
SOURCE_IP="${SOURCE_IP:-192.168.1.105}"
CHECK_FACTORY_TOPICS="${CHECK_FACTORY_TOPICS:-true}"

# 设备直连和组播实验不能与被动模式共存，因为它们可能占用端口/设备，
# 或意味着原厂 ROS1 数据源已被停止。
for legacy_name in \
  x30_ros2_sensors \
  x30_livox_ros2 \
  x30_cloud_merger \
  x30_body_imu \
  x30_ros2_livox_slave
do
  if docker ps --format '{{.Names}}' | grep -qx "${legacy_name}"; then
    echo "[passive-run] ERROR: legacy container ${legacy_name} is still running." >&2
    echo "[passive-run] Stop the old Docker sensor mode and restore the factory ROS1 sensors first." >&2
    echo "[passive-run] No container was stopped automatically." >&2
    exit 1
  fi
done

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[passive-run] ERROR: image ${IMAGE} is not present." >&2
  echo "[passive-run] Build it first with: bash remote/00_build_image.sh" >&2
  exit 1
fi

# 健康容器重复执行时保持幂等；同名停止容器会先删除，再创建目标运行实例。
if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[passive-run] ${NAME} is already running."
  echo "[passive-run] check: bash remote/27_check_passive_relay.sh"
  exit 0
fi
if docker container inspect "${NAME}" >/dev/null 2>&1; then
  docker rm -f "${NAME}" >/dev/null
fi

# host 网络会让监听器直接绑定到 106。端口冲突时立即失败，
# 不与其他转发实例静默共用。
if ss -ltn 2>/dev/null | grep -Eq ':(56110|56111|56112)[[:space:]]'; then
  echo "[passive-run] ERROR: one or more passive relay TCP ports are occupied." >&2
  ss -ltnp 2>/dev/null | grep -E ':(56110|56111|56112)[[:space:]]' >&2 || true
  exit 1
fi

if [[ "${CHECK_FACTORY_TOPICS}" == "true" ]]; then
  source /opt/ros/noetic/setup.bash
  source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
  source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

  # 打开被动监听前，确认 105 的 ROS1 图正在产生全部必需数据。
  # 以下命令只进行订阅。
  echo "[passive-run] checking factory ROS1 sources without modifying them..."
  for topic in /lidar_points /imu/data /leg_odom; do
    if ! timeout 5 rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1; then
      echo "[passive-run] ERROR: ${topic} has no fresh message." >&2
      echo "[passive-run] Keep the factory sensor and LIO chain running before passive startup." >&2
      exit 1
    fi
  done
fi

echo "[passive-run] starting ${NAME} from ${IMAGE}..."
echo "[passive-run] no sensor device is mounted and no factory process is stopped."
# host 网络暴露三路 TCP 监听。复制传感器消息不需要设备映射、
# 提升容器权限或可写 host 挂载。
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  --log-opt max-size=20m \
  --log-opt max-file=3 \
  "${IMAGE}" \
  ros2 launch x30_sensor_receiver passive_receiver.launch.py \
    bind_address:=0.0.0.0 \
    allowed_source_ip:="${SOURCE_IP}" \
    point_cloud_port:=56110 \
    imu_port:=56111 \
    odometry_port:=56112 \
    point_cloud_topic:=/x30/lidar_points \
    imu_topic:=/x30/body_imu \
    odometry_topic:=/x30/leg_odom

sleep 2
# 容器启动后持续运行，说明三路监听已完成绑定；消息新鲜度由
# 27_check_passive_relay.sh 单独检查。
if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[passive-run] ERROR: ${NAME} exited during startup." >&2
  docker logs "${NAME}" >&2 || true
  exit 1
fi

echo "[passive-run] receiver is listening for 105 on TCP 56110-56112."
echo "[passive-run] next on 105: bash remote/02_start_forwarder_105.sh"
echo "[passive-run] check: bash remote/27_check_passive_relay.sh"
