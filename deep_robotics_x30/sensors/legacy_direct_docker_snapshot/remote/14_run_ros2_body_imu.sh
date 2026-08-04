#!/bin/bash
set -euo pipefail

IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
NAME="${NAME:-x30_body_imu}"
DEVICE="${DEVICE:-/dev/ttyS0}"
TOPIC="${TOPIC:-/x30/body_imu}"
RAW_TOPIC="${RAW_TOPIC:-/x30/body_imu_raw}"
FRAME_ID="${FRAME_ID:-imu_link}"
BAUD_RATE="${BAUD_RATE:-115200}"

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

if rosnode list 2>/dev/null | grep -q '^/yesense_imu_node$'; then
  echo "[imu] ERROR: factory ROS1 Yesense IMU is still running." >&2
  echo "[imu] Run: bash remote/11_factory_imu_off.sh" >&2
  exit 1
fi

if sudo lsof "${DEVICE}" >/tmp/x30_body_imu_lsof.txt 2>/dev/null; then
  echo "[imu] ERROR: ${DEVICE} is still in use:" >&2
  cat /tmp/x30_body_imu_lsof.txt >&2
  exit 1
fi

docker rm -f "${NAME}" >/dev/null 2>&1 || true

echo "[imu] starting ${NAME} from ${IMAGE} on ${DEVICE}..."
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  --privileged \
  --device="${DEVICE}:${DEVICE}" \
  "${IMAGE}" \
  ros2 run yesense_std_ros2 yesense_node_publisher --ros-args \
    -p serial_port:="${DEVICE}" \
    -p baud_rate:="${BAUD_RATE}" \
    -p frame_id:="${FRAME_ID}" \
    -p driver_type:="linux_serial" \
    -p imu_topic_ros:="${TOPIC}" \
    -p imu_topic:="${RAW_TOPIC}"

echo "[imu] logs: docker logs -f ${NAME}"
echo "[imu] output topic: ${TOPIC}"
