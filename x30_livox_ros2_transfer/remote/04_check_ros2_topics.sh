#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_livox_ros2}"

docker exec -it "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo "=== ROS2 topics ==="
ros2 topic list
echo
echo "=== /livox/lidar hz ==="
timeout 12 ros2 topic hz /livox/lidar || true
echo
echo "=== /livox/imu hz ==="
timeout 12 ros2 topic hz /livox/imu || true
'

