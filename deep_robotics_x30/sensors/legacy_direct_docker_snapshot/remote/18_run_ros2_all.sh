#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
NAME="${NAME:-x30_ros2_sensors}"
DEVICE="${DEVICE:-/dev/ttyS0}"
ENABLE_BODY_IMU="${ENABLE_BODY_IMU:-true}"
CONFIG_PATH="${CONFIG_PATH:-/config/x30_multi_mid360_ros2.json}"

echo "[all] stopping old split ROS2 containers if they exist..."
docker rm -f "${NAME}" x30_livox_ros2 x30_cloud_merger x30_body_imu >/dev/null 2>&1 || true

echo "[all] checking Livox UDP ports..."
if ss -unlp | egrep -q '56101|56201|56301|56401|56501|224.1.1.5'; then
  echo "[all] ERROR: Livox UDP ports are still in use. Run remote/02_factory_livox_off.sh first." >&2
  ss -unlp | egrep '56101|56201|56301|56401|56501|224.1.1.5' || true
  exit 1
fi

if [ "${ENABLE_BODY_IMU}" = "true" ]; then
  source /opt/ros/noetic/setup.bash
  source /home/ysc/jy_cog/drivers/setup.bash

  if rosnode list 2>/dev/null | grep -q '^/yesense_imu_node$'; then
    echo "[all] ERROR: factory ROS1 Yesense IMU is still running." >&2
    echo "[all] Run: bash remote/11_factory_imu_off.sh" >&2
    exit 1
  fi

  if sudo lsof "${DEVICE}" >/tmp/x30_ros2_sensors_lsof.txt 2>/dev/null; then
    echo "[all] ERROR: ${DEVICE} is still in use:" >&2
    cat /tmp/x30_ros2_sensors_lsof.txt >&2
    exit 1
  fi
fi

echo "[all] starting ${NAME} from ${IMAGE}..."
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  --privileged \
  --device="${DEVICE}:${DEVICE}" \
  -v "${ROOT}/config:/config:ro" \
  "${IMAGE}" \
  ros2 launch x30_livox_tools x30_all_sensors_launch.py \
    config_path:="${CONFIG_PATH}" \
    xfer_format:=0 \
    multi_topic:=0 \
    publish_freq:=10.0 \
    lidar_frame_id:=lidar_link \
    cloud_input_topic:=/livox/lidar \
    cloud_output_topic:=/x30/points_merged \
    cloud_output_frame:=lidar_link \
    cloud_window_ms:=100.0 \
    cloud_min_clouds:=1 \
    enable_body_imu:="${ENABLE_BODY_IMU}" \
    serial_port:="${DEVICE}" \
    baud_rate:=115200 \
    imu_frame_id:=imu_link \
    imu_topic_ros:=/x30/body_imu \
    imu_topic_raw:=/x30/body_imu_raw

echo "[all] logs: docker logs -f ${NAME}"
echo "[all] shell: docker exec -it ${NAME} bash"
