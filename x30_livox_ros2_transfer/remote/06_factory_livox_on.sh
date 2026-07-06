#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

if rosnode list 2>/dev/null | grep -q 'livox_lidar_publisher'; then
  echo "[factory] ROS1 Livox driver is already running."
else
  echo "[factory] starting ROS1 Livox driver..."
  nohup bash /home/ysc/jy_cog/drivers/scripts/lidar_driver.sh livox \
    > /tmp/factory_livox_driver.log 2>&1 &
  sleep 4
fi

echo "[factory] checking /lidar_points and /imu/data..."
timeout 8 rostopic hz /lidar_points -w 3 || true
timeout 8 rostopic hz /imu/data -w 3 || true

