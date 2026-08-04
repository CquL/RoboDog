#!/bin/bash
set -euo pipefail

# 显式实机控制边界。离线链接检查通过后，使用 host 网络初始化 X30 HAL，
# 且只发送零速度。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
CONFIG_PATH="${CONFIG_PATH:-${ROOT}/components/robot_hardware_x30/config.yaml}"
readonly CONFIRMATION="CONFIRM_X30_ZERO_ONLY"

if [[ "${1:-}" != "${CONFIRMATION}" ]]; then
  echo "Usage: bash remote/31_test_x30_hal_zero.sh ${CONFIRMATION}" >&2
  echo "This initializes the X30 UDP HAL and sends zero velocity only." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[hal-zero] ERROR: image ${IMAGE} is not present." >&2
  exit 1
fi
if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "[hal-zero] ERROR: config not found: ${CONFIG_PATH}" >&2
  exit 1
fi
# 零速度验证不得更改控制器模式或速度源，因此要求两个自动配置选项均关闭。
if ! grep -Eq '^[[:space:]]*configure_non_manual_mode:[[:space:]]*false' \
    "${CONFIG_PATH}" ||
   ! grep -Eq '^[[:space:]]*configure_navigation_velocity_source:[[:space:]]*false' \
    "${CONFIG_PATH}"; then
  echo "[hal-zero] ERROR: both automatic mode/source configuration flags must be false." >&2
  exit 1
fi

# 防止两个本地命令源同时竞争 X30 UDP 端点。
if pgrep -af 'robot_test_x30|x30_hardware_controller' |
    grep -vE '31_test_x30_hal_zero|pgrep -af' >/dev/null; then
  echo "[hal-zero] ERROR: another X30 test/controller process is running." >&2
  pgrep -af 'robot_test_x30|x30_hardware_controller' >&2 || true
  exit 1
fi

echo "[hal-zero] Image: ${IMAGE}"
echo "[hal-zero] Config: ${CONFIG_PATH}"
echo "[hal-zero] This sends heartbeat, connection confirmation, and zero velocity."
echo "[hal-zero] It sends no gait switch and no nonzero velocity."

echo "[hal-zero] Verifying the HAL library with --network none first..."
# 第一个容器在严格无网络边界下验证链接。
docker run --rm \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,size=16m \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --entrypoint bash \
  "${IMAGE}" -lc '
set -euo pipefail
prefix=/opt/x30_robot_hardware
expected_library="${prefix}/lib/librobot_hardware_x30.so"
test -f "${expected_library}"
loader_output="$(ldd "${prefix}/bin/robot_test_x30")"
resolved_library="$(
  awk '"'"'$1 == "librobot_hardware_x30.so" {print $3}'"'"' \
    <<<"${loader_output}"
)"
if [[ -z "${resolved_library}" || ! -e "${resolved_library}" ]]; then
  printf "%s\n" "${loader_output}" >&2
  echo "[hal-zero] ERROR: installed HAL library was not resolved." >&2
  exit 1
fi
expected_library="$(readlink -f "${expected_library}")"
resolved_library="$(readlink -f "${resolved_library}")"
if [[ "${resolved_library}" != "${expected_library}" ]]; then
  printf "%s\n" "${loader_output}" >&2
  echo "[hal-zero] ERROR: wrong HAL library: ${resolved_library:-missing}" >&2
  exit 1
fi
'

# 只有第二个经显式确认的容器使用 host 网络，
# 其 entrypoint 为统一测试程序的 zero 模式。
docker run --rm \
  --network host \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,size=16m \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --mount "type=bind,src=${CONFIG_PATH},dst=/config/robot_hardware.yaml,readonly" \
  --entrypoint /opt/x30_robot_hardware/bin/robot_test_x30 \
  "${IMAGE}" \
  /config/robot_hardware.yaml zero

echo "[hal-zero] Docker X30 HAL zero-only test finished."
