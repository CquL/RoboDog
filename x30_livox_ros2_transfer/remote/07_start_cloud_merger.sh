#!/bin/bash
set -euo pipefail

IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
LIVOX_NAME="${LIVOX_NAME:-x30_livox_ros2}"
NAME="${NAME:-x30_cloud_merger}"
INPUT_TOPIC="${INPUT_TOPIC:-/livox/lidar}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/x30/points_merged}"
OUTPUT_FRAME="${OUTPUT_FRAME:-lidar_link}"
WINDOW_MS="${WINDOW_MS:-100.0}"
MIN_CLOUDS="${MIN_CLOUDS:-1}"

if ! docker ps --format '{{.Names}}' | grep -qx "${LIVOX_NAME}"; then
  echo "[merge] ERROR: container ${LIVOX_NAME} is not running. Start ROS2 Livox first." >&2
  exit 1
fi

docker rm -f "${NAME}" >/dev/null 2>&1 || true

echo "[merge] starting ${NAME} from ${IMAGE}..."
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  --privileged \
  "${IMAGE}" \
  ros2 run x30_pointcloud_tools time_window_cloud_merger --ros-args \
    -p input_topic:="${INPUT_TOPIC}" \
    -p output_topic:="${OUTPUT_TOPIC}" \
    -p output_frame:="${OUTPUT_FRAME}" \
    -p window_ms:="${WINDOW_MS}" \
    -p min_clouds:="${MIN_CLOUDS}"

echo "[merge] logs: docker logs -f ${NAME}"
echo "[merge] output topic: ${OUTPUT_TOPIC}"
