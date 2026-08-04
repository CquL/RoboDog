#!/bin/bash
set -euo pipefail

# 只启动一个后台 ROS1 转发节点，并记录 roslaunch PID。该进程仅订阅；
# 105 上的传感器驱动和原厂地形节点保持原状运行。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${ROOT}/catkin_ws"
RUNTIME_DIR="${HOME}/.x30_sensor_forwarder"
PID_FILE="${RUNTIME_DIR}/forwarder.pid"
LOG_FILE="${RUNTIME_DIR}/forwarder.log"
RECEIVER_HOST="${RECEIVER_HOST:-192.168.1.106}"

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true
source "${WORKSPACE}/devel/setup.bash"

mkdir -p "${RUNTIME_DIR}"

is_forwarder_process() {
  # PID 文件不能单独证明进程归属，必须先匹配命令行。
  local candidate_pid="$1"
  ps -p "${candidate_pid}" -o args= 2>/dev/null |
    grep -Eq 'roslaunch.*x30_sensor_forwarder_ros1.*forwarder\.launch'
}

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}")"
  if [[ "${old_pid}" =~ ^[0-9]+$ ]] &&
     (( old_pid > 1 )) &&
     kill -0 "${old_pid}" 2>/dev/null &&
     is_forwarder_process "${old_pid}"; then
    echo "[105-start] forwarder is already running with PID ${old_pid}."
    exit 0
  fi
  echo "[105-start] removing stale PID file for ${old_pid}; no signal was sent."
  rm -f "${PID_FILE}"
fi

echo "[105-start] destination: ${RECEIVER_HOST}"
echo "[105-start] this node only subscribes to ROS1 sensor topics."
echo "[105-start] it does not publish ROS1 topics or call services."

nohup roslaunch x30_sensor_forwarder_ros1 forwarder.launch \
  receiver_host:="${RECEIVER_HOST}" \
  >"${LOG_FILE}" 2>&1 &
pid=$!
echo "${pid}" >"${PID_FILE}"

# roslaunch 提前退出或记录的 PID 被意外复用时立即失败。
sleep 2
if ! kill -0 "${pid}" 2>/dev/null; then
  echo "[105-start] ERROR: forwarder exited during startup." >&2
  tail -n 100 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  exit 1
fi
if ! is_forwarder_process "${pid}"; then
  echo "[105-start] ERROR: PID ${pid} is not the expected roslaunch process." >&2
  tail -n 100 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  exit 1
fi

echo "[105-start] running with PID ${pid}."
echo "[105-start] log: ${LOG_FILE}"
