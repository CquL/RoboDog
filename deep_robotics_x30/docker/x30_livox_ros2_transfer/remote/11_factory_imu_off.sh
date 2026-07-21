#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

echo "[factory] stopping ROS1 Yesense IMU driver..."

if [ -x /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh ]; then
  bash /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh || true
elif [ -f /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh ]; then
  bash /home/ysc/jy_cog/drivers/scripts/imu_driver_stop.sh || true
else
  echo "[factory] no imu_driver_stop.sh found; killing ROS node and known process names..."
  rosnode kill /yesense_imu_node 2>/dev/null || true
  pkill -f '/home/ysc/jy_cog/drivers/lib/yesense_imu/yesense_imu_node' 2>/dev/null || true
  pkill -f 'roslaunch yesense_imu yesense.launch' 2>/dev/null || true
  pkill -f '/home/ysc/jy_cog/drivers/scripts/imu_driver.sh' 2>/dev/null || true
fi

sleep 2

echo "[factory] remaining ROS1 IMU nodes:"
rosnode list 2>/dev/null | grep -Ei 'imu|yesense' || true

echo
echo "[factory] remaining ROS1 IMU processes:"
ps -ef | grep -Ei 'imu|yesense|serial' | grep -v grep || true
