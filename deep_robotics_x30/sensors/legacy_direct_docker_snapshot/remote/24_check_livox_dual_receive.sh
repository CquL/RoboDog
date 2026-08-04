#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_ros2_livox_slave}"

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[dual-check] ERROR: ${NAME} is not running." >&2
  exit 1
fi

echo "=== Factory ROS1 /lidar_points ==="
timeout 12 rostopic hz /lidar_points || true

echo
echo "=== Docker ROS2 /livox/lidar and /x30/points_merged ==="
docker exec "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo "--- /livox/lidar ---"
timeout 12 ros2 topic hz /livox/lidar || true
echo "--- /x30/points_merged ---"
timeout 12 ros2 topic hz /x30/points_merged || true
'

echo
echo "[dual-check] Review both measured rates before continuing."
