#!/bin/bash
set -euo pipefail

# 只删除 106 上指定的被动容器。105 上的原厂 ROS1 传感器、LIO、
# 地形和 gridmap 进程不在本脚本管理范围内。
readonly NAME="x30_ros2_passive"

if ! docker info >/dev/null 2>&1; then
  echo "[passive-stop] ERROR: Docker daemon is unavailable." >&2
  exit 1
fi

if ! docker container inspect "${NAME}" >/dev/null 2>&1; then
  echo "[passive-stop] ${NAME} is already absent."
  echo "[passive-stop] no factory ROS1 process was changed."
  exit 0
fi

# Docker 停止/删除会关闭三路监听，不向 105 转发端发送远程命令；
# 此容器再次启动后，105 可重新连接。
if ! docker rm -f "${NAME}" >/dev/null; then
  echo "[passive-stop] ERROR: failed to remove ${NAME}." >&2
  exit 1
fi

if docker container inspect "${NAME}" >/dev/null 2>&1; then
  echo "[passive-stop] ERROR: ${NAME} still exists after docker rm." >&2
  exit 1
fi

echo "[passive-stop] stopped and removed ${NAME}."
echo "[passive-stop] no factory ROS1 process was changed."
