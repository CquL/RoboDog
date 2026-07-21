#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_body_imu}"
TOPIC="${TOPIC:-/x30/body_imu}"

docker exec -it "${NAME}" bash -lc "
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo '=== ROS2 topics ==='
ros2 topic list | sort
echo
echo '=== body IMU topic info ==='
ros2 topic info ${TOPIC} -v
echo
echo '=== body IMU one message ==='
timeout 5 ros2 topic echo --once ${TOPIC} || true
echo
echo '=== body IMU hz ==='
timeout 10 ros2 topic hz ${TOPIC} || true
"
