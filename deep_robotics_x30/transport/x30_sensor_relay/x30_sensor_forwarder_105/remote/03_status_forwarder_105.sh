#!/bin/bash
set -euo pipefail

# 仅用于观测：确认进程身份、ROS 订阅、到 106 的三路 TCP 会话，
# 并查看日志中的近期丢帧和重连计数。
RUNTIME_DIR="${HOME}/.x30_sensor_forwarder"
PID_FILE="${RUNTIME_DIR}/forwarder.pid"
LOG_FILE="${RUNTIME_DIR}/forwarder.log"

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

is_forwarder_process() {
  local candidate_pid="$1"
  ps -p "${candidate_pid}" -o args= 2>/dev/null |
    grep -Eq 'roslaunch.*x30_sensor_forwarder_ros1.*forwarder\.launch'
}

if [[ ! -f "${PID_FILE}" ]]; then
  echo "[105-status] forwarder is not running; PID file is absent."
  exit 1
fi

# 不把过期或无关 PID 报告为健康转发进程。
pid="$(cat "${PID_FILE}")"
if ! [[ "${pid}" =~ ^[0-9]+$ ]] ||
   (( pid <= 1 )) ||
   ! kill -0 "${pid}" 2>/dev/null ||
   ! is_forwarder_process "${pid}"; then
  echo "[105-status] forwarder is not running; stale PID ${pid}."
  exit 1
fi

echo "[105-status] forwarder PID: ${pid}"
ps -p "${pid}" -o pid,etime,%cpu,%mem,args

echo
echo "=== ROS1 node ==="
rosnode info /x30_sensor_forwarder || true

echo
echo "=== TCP connections ==="
# 三个独立端口依次对应点云、IMU 和里程计。
ss -tnp 2>/dev/null |
  grep -E '192\.168\.1\.106:(56110|56111|56112)' || true

echo
echo "=== recent relay log ==="
tail -n 40 "${LOG_FILE}" 2>/dev/null || true
