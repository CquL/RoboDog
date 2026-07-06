#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

echo "=== ROS1 IMU topic ==="
rostopic info /imu/data || true

echo
echo "=== ROS1 IMU hz ==="
timeout 8 rostopic hz /imu/data -w 3 || true

echo
echo "=== ROS1 IMU nodes ==="
rosnode list 2>/dev/null | grep -Ei 'imu|yesense' || true

echo
echo "=== ROS1 IMU processes ==="
ps -ef | grep -Ei 'imu|yesense|serial' | grep -v grep || true

echo
echo "=== factory IMU scripts ==="
ls -l /home/ysc/jy_cog/drivers/scripts/*imu* 2>/dev/null || true

echo
echo "=== yesense package/config hints ==="
rospack find yesense_imu 2>/dev/null || true
rosparam list 2>/dev/null | grep -Ei 'imu|yesense|serial|baud|port' || true

echo
echo "=== serial devices ==="
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* /dev/serial/by-id 2>/dev/null || true

echo
echo "=== serial users ==="
sudo lsof /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null || true
