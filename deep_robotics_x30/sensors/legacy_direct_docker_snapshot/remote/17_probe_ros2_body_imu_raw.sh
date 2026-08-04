#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_body_imu}"

if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[imu-probe] ERROR: container ${NAME} is not running." >&2
  echo "[imu-probe] Run first: bash remote/14_run_ros2_body_imu.sh" >&2
  exit 1
fi

docker exec -it "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

echo "=== related ROS2 topics ==="
ros2 topic list | sort | grep -E "body_imu|imu_raw|euler|att_|robot_lord|pose_geo|marker" || true
echo

echo "=== standard IMU ==="
ros2 topic info /x30/body_imu -v || true
timeout 3 ros2 topic echo --once /x30/body_imu || true
echo

for topic in /x30/body_imu_raw /euler_only /robot_lord /att_min_vru /att_min_ahrs /att_all /pose_geo; do
  echo "===== ${topic} info ====="
  ros2 topic info "${topic}" -v || true
  echo "===== ${topic} one msg ====="
  timeout 3 ros2 topic echo --once "${topic}" || true
  echo
done
'
