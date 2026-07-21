#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_ros2_sensors}"

if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[all-check] ERROR: container ${NAME} is not running." >&2
  echo "[all-check] Run first: bash remote/18_run_ros2_all.sh" >&2
  exit 1
fi

docker exec -it "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

echo "=== ROS2 nodes ==="
ros2 node list | sort
echo

echo "=== ROS2 topics ==="
ros2 topic list | sort | grep -E "livox|x30|body_imu|euler|att_|robot_lord" || true
echo

echo "=== /livox/lidar ==="
ros2 topic info /livox/lidar -v || true
timeout 8 ros2 topic hz /livox/lidar || true
echo

echo "=== /livox/imu ==="
ros2 topic info /livox/imu -v || true
timeout 8 ros2 topic hz /livox/imu || true
echo

echo "=== /x30/points_merged ==="
ros2 topic info /x30/points_merged -v || true
timeout 8 ros2 topic hz /x30/points_merged || true
timeout 5 ros2 topic echo --once /x30/points_merged --field header || true
echo

echo "=== /x30/body_imu ==="
ros2 topic info /x30/body_imu -v || true
timeout 8 ros2 topic hz /x30/body_imu || true
timeout 5 ros2 topic echo --once /x30/body_imu || true
'
