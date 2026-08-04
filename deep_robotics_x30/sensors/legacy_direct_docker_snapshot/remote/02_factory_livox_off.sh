#!/bin/bash
set -euo pipefail

echo "[factory] stopping ROS1 Livox driver..."
bash /home/ysc/jy_cog/drivers/scripts/lidar_driver_stop.sh livox || true
sleep 2

echo "[factory] remaining Livox UDP ports:"
ss -unlp | egrep '56101|56201|56301|56401|56501|224.1.1.5' || true

echo "[factory] remaining ROS1 livox nodes:"
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash
rosnode list 2>/dev/null | grep -i livox || true

