#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

if rosnode list 2>/dev/null | grep -q '^/yesense_imu_node$'; then
  echo "[factory] ROS1 Yesense IMU driver is already running."
else
  echo "[factory] starting ROS1 Yesense IMU driver..."
  nohup bash /home/ysc/jy_cog/drivers/scripts/imu_driver.sh \
    > /tmp/factory_imu_driver.log 2>&1 &
  sleep 4
fi

echo "[factory] checking /imu/data..."
rostopic info /imu/data || true
timeout 8 rostopic hz /imu/data -w 3 || true
