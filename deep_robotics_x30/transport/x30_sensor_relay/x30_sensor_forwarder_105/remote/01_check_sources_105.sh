#!/bin/bash
set -euo pipefail

# 原厂 ROS1 发布者的只读预检：检查图、频率、header 和 PointCloud2 结构，
# 不发布消息、不调用服务，可在启动转发前安全执行。
source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

echo "[105-check] read-only source validation; no service or command is sent."

# 106 被动数据接口需要以下三路数据。
for topic in /lidar_points /imu/data /leg_odom; do
  echo
  echo "=== ${topic} ==="
  rostopic info "${topic}"
  timeout 5 rostopic hz "${topic}" || true
done

echo
echo "=== point cloud contract ==="
# 通过 header 和 fields 检查数据源是否仍符合预期的 lidar_link 坐标系及
# x/y/z/intensity/tag/line/timestamp 字段结构。
rostopic type /lidar_points
timeout 5 rostopic echo -n 1 /lidar_points/header || true
timeout 5 rostopic echo -n 1 /lidar_points/fields || true

echo
echo "[105-check] source validation finished."
