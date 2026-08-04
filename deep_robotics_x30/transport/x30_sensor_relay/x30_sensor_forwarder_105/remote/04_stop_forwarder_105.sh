#!/bin/bash
set -euo pipefail

# 只停止本包此前记录的 roslaunch 进程。不使用 pkill、不触碰原厂 ROS1
# 节点，也不会自动升级到 SIGKILL。
RUNTIME_DIR="${HOME}/.x30_sensor_forwarder"
PID_FILE="${RUNTIME_DIR}/forwarder.pid"

is_forwarder_process() {
  local candidate_pid="$1"
  ps -p "${candidate_pid}" -o args= 2>/dev/null |
    grep -Eq 'roslaunch.*x30_sensor_forwarder_ros1.*forwarder\.launch'
}

if [[ ! -f "${PID_FILE}" ]]; then
  echo "[105-stop] forwarder is already stopped."
  exit 0
fi

pid="$(cat "${PID_FILE}")"
if ! [[ "${pid}" =~ ^[0-9]+$ ]] || (( pid <= 1 )); then
  echo "[105-stop] refusing invalid PID file content: ${pid}" >&2
  rm -f "${PID_FILE}"
  exit 1
fi
if ! kill -0 "${pid}" 2>/dev/null; then
  echo "[105-stop] removing stale PID file for ${pid}."
  rm -f "${PID_FILE}"
  exit 0
fi
if ! is_forwarder_process "${pid}"; then
  echo "[105-stop] refusing to signal PID ${pid}; it is not our forwarder." >&2
  echo "[105-stop] removing only the stale PID file." >&2
  rm -f "${PID_FILE}"
  exit 1
fi

# SIGINT 为 roslaunch 和 C++ 工作线程提供正常 ROS 关闭路径。
echo "[105-stop] stopping forwarder PID ${pid}..."
if ! kill -INT "${pid}" 2>/dev/null; then
  rm -f "${PID_FILE}"
  echo "[105-stop] process exited before SIGINT was delivered."
  exit 0
fi

for _ in $(seq 1 30); do
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "[105-stop] stopped."
    exit 0
  fi
  sleep 0.2
done

echo "[105-stop] SIGINT timeout; sending SIGTERM to our recorded PID ${pid}."
kill -TERM "${pid}" 2>/dev/null || true

for _ in $(seq 1 20); do
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "[105-stop] stopped after SIGTERM."
    exit 0
  fi
  sleep 0.2
done

echo "[105-stop] ERROR: PID ${pid} is still running; no SIGKILL was sent." >&2
exit 1
