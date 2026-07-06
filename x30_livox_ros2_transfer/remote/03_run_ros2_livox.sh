#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
NAME="${NAME:-x30_livox_ros2}"

echo "[ros2] checking Livox UDP ports..."
if ss -unlp | egrep -q '56101|56201|56301|56401|56501|224.1.1.5'; then
  echo "[ros2] ERROR: Livox UDP ports are still in use. Run remote/02_factory_livox_off.sh first." >&2
  ss -unlp | egrep '56101|56201|56301|56401|56501|224.1.1.5' || true
  exit 1
fi

docker rm -f "${NAME}" >/dev/null 2>&1 || true

echo "[ros2] starting ${NAME} from ${IMAGE}..."
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  --privileged \
  -v "${ROOT}/config:/config:ro" \
  "${IMAGE}" \
  ros2 launch x30_livox_bringup x30_mid360_launch.py \
    config_path:=/config/x30_multi_mid360_ros2.json \
    xfer_format:=0 \
    multi_topic:=0 \
    publish_freq:=10.0 \
    frame_id:=lidar_link

echo "[ros2] logs: docker logs -f ${NAME}"
echo "[ros2] shell: docker exec -it ${NAME} bash"
