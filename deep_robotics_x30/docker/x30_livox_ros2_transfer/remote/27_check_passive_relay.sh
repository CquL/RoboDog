#!/bin/bash
set -euo pipefail

# 106 端到端健康检查：container -> host TCP socket -> ROS2 图 -> 新鲜消息。
# 不执行 ROS 发布或机器人控制动作。
readonly NAME="x30_ros2_passive"

if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[passive-check] ERROR: container ${NAME} is not running." >&2
  echo "[passive-check] Run first: bash remote/26_run_passive_relay.sh" >&2
  exit 1
fi

echo "=== container ==="
docker ps --filter "name=^/${NAME}$" \
  --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'

echo
echo "=== host TCP listeners/connections ==="
# LISTEN 表示接收端已监听三路端口，ESTAB 表示 105 已建立连接。
ss -ltnp 2>/dev/null |
  grep -E ':(56110|56111|56112)[[:space:]]' || true
ss -tnp 2>/dev/null |
  grep -E '192\.168\.1\.(105|106):(56110|56111|56112)' || true

docker exec "${NAME}" bash -lc '
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

echo
echo "=== ROS2 node and topics ==="
ros2 node list | grep -x "/x30_sensor_receiver"
ros2 topic list | grep -E "^/x30/(lidar_points|body_imu|leg_odom)$" | sort

# 频率输出仅用于诊断，读取单条消息才是新鲜度门槛。
# best_effort 与接收端发布者一致，可避免 QoS 不匹配。
for topic in /x30/lidar_points /x30/body_imu /x30/leg_odom; do
  echo
  echo "=== ${topic} ==="
  ros2 topic info "${topic}" -v
  timeout 8 ros2 topic hz "${topic}" || true
  if ! timeout 8 ros2 topic echo \
      --qos-reliability best_effort \
      --once "${topic}" --field header; then
    echo "[passive-check] ERROR: ${topic} has no fresh message." >&2
    exit 1
  fi
done

echo
echo "=== point cloud fields ==="
# 结构检查用于确认原始 PointCloud2 字段经过 ROS1 序列化和 ROS2 重建后
# 保持完整，不输出庞大的 data 数组。
timeout 5 ros2 topic echo \
  --qos-reliability best_effort \
  --once /x30/lidar_points --field fields || true
'

echo
echo "=== recent receiver log ==="
docker logs --tail 60 "${NAME}" 2>&1 || true
