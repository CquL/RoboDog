#!/bin/bash
set -euo pipefail

# 日常使用的轻量状态查看。与完整检查脚本不同，不订阅 Topic，
# 也不等待频率采样。
readonly NAME="x30_ros2_passive"

if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[passive-status] ${NAME} is not running."
  docker ps -a --filter "name=^/${NAME}$" \
    --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}' || true
  exit 1
fi

docker ps --filter "name=^/${NAME}$" \
  --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}'

echo
# 快速查看只检查监听器和近期接收计数；需要证明消息新鲜度时使用脚本 27。
ss -ltnp 2>/dev/null |
  grep -E ':(56110|56111|56112)[[:space:]]' || true

echo
docker logs --tail 30 "${NAME}" 2>&1 || true
