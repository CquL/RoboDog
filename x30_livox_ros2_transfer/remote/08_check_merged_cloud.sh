#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_cloud_merger}"
TOPIC="${TOPIC:-/x30/points_merged}"

docker exec -it "${NAME}" bash -lc "
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo '=== merged topic info ==='
ros2 topic info ${TOPIC} -v
echo
echo '=== merged topic header ==='
timeout 5 ros2 topic echo --once ${TOPIC} sensor_msgs/msg/PointCloud2 --field header || true
echo
echo '=== merged topic width ==='
timeout 5 ros2 topic echo --once ${TOPIC} sensor_msgs/msg/PointCloud2 --field width || true
echo
echo '=== merged topic hz ==='
timeout 12 ros2 topic hz ${TOPIC} || true
"
