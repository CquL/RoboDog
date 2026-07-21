#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
NAME="${NAME:-x30_ros2_livox_slave}"
CONFIG_PATH="${CONFIG_PATH:-/config/x30_multi_mid360_ros2_slave.json}"

cleanup_on_error() {
  status=$?
  if [ "${status}" -ne 0 ]; then
    echo "[slave-run] Failed. Removing ${NAME}; factory ROS1 was not stopped." >&2
    docker logs --tail 120 "${NAME}" 2>/dev/null || true
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
  fi
  exit "${status}"
}
trap cleanup_on_error EXIT

echo "[slave-run] Starting ROS2 as Livox multicast slave."
echo "[slave-run] Factory ROS1 remains the only Livox master."
echo "[slave-run] This script sends no robot motion command."

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

if docker ps --format '{{.Names}}' | grep -Eq '^(x30_ros2_sensors|x30_livox_ros2|x30_cloud_merger)$'; then
  echo "[slave-run] ERROR: another ROS2 Livox container is already running." >&2
  docker ps --format 'table {{.Names}}\t{{.Status}}' | grep -E 'x30_ros2_sensors|x30_livox_ros2|x30_cloud_merger' || true
  exit 1
fi

if ! rosnode list 2>/dev/null | grep -qi livox; then
  echo "[slave-run] ERROR: factory ROS1 Livox node is not running." >&2
  echo "[slave-run] Restore it first with: bash remote/06_factory_livox_on.sh" >&2
  exit 1
fi

if ! timeout 8 rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; then
  echo "[slave-run] ERROR: factory /lidar_points has no data." >&2
  exit 1
fi

docker run --rm --entrypoint bash "${IMAGE}" -lc '
sdk=/usr/local/lib/liblivox_lidar_sdk_shared.so
test -f "${sdk}" && grep -aFq "master_sdk" "${sdk}"
'

docker rm -f "${NAME}" >/dev/null 2>&1 || true
docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
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
    enable_body_imu:=false >/dev/null

sleep 5
if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[slave-run] ERROR: ${NAME} exited during startup." >&2
  exit 1
fi

echo "[slave-run] Waiting for one ROS2 Livox cloud..."
docker exec "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
timeout 12 ros2 topic echo --once /livox/lidar --field header >/dev/null
'

echo "[slave-run] Confirming factory ROS1 still receives clouds..."
if ! timeout 8 rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; then
  echo "[slave-run] ERROR: ROS1 /lidar_points stopped after slave startup." >&2
  exit 1
fi

trap - EXIT
echo "[slave-run] PASS: ROS1 and ROS2 both received Livox data."
echo "[slave-run] Check frequencies: bash remote/24_check_livox_dual_receive.sh"
echo "[slave-run] Stop slave:       bash remote/23_stop_ros2_livox_slave.sh"
