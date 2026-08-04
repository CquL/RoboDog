#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash

echo "=== yesense package path ==="
PKG="$(rospack find yesense_imu 2>/dev/null || true)"
echo "${PKG}"

echo
echo "=== yesense launch files ==="
if [ -n "${PKG}" ]; then
  find "${PKG}" -maxdepth 3 -type f \( -name '*.launch' -o -name '*.yaml' -o -name '*.xml' \) -print -exec sed -n '1,220p' {} \; || true
fi

echo
echo "=== jy_cog yesense related files ==="
find /home/ysc/jy_cog/drivers -maxdepth 5 -type f 2>/dev/null | grep -Ei 'yesense|imu' | sort || true

echo
echo "=== grep serial/baud/port hints ==="
grep -RInE 'tty|serial|baud|port|device|frame' /home/ysc/jy_cog/drivers 2>/dev/null | grep -Ei 'yesense|imu|tty|baud|serial' | head -200 || true

echo
echo "=== params under ROS master ==="
rosparam list 2>/dev/null | grep -Ei 'imu|yesense|serial|baud|port|device|frame' | while read -r p; do
  echo "--- ${p}"
  rosparam get "${p}" 2>/dev/null || true
done
