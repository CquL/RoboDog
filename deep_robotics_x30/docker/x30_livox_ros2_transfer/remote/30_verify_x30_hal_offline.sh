#!/bin/bash
set -euo pipefail

# 在无网络环境下验证已安装的 X30 HAL 文件、协议测试和动态链接。
# 本脚本独立于被动传感器数据链。
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
readonly PREFIX="/opt/x30_robot_hardware"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[hal-offline] ERROR: image ${IMAGE} is not present." >&2
  exit 1
fi

echo "[hal-offline] Running with --network none."
echo "[hal-offline] No robot UDP packet can leave this container."
# 只读根目录、删除 capabilities 和 no-new-privileges 共同确保本步骤
# 仅进行打包/ABI 检查，而不是实机测试。
docker run --rm \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,size=16m \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --entrypoint bash \
  --env "X30_ROBOT_HARDWARE_PREFIX=${PREFIX}" \
  "${IMAGE}" -lc '
set -euo pipefail
prefix="${X30_ROBOT_HARDWARE_PREFIX}"
expected_library="${prefix}/lib/librobot_hardware_x30.so"
test -f "${expected_library}"
test -f "${prefix}/include/robot_hardware/robot_hardware_interface.h"
test -x "${prefix}/bin/robot_test_x30"
"${prefix}/bin/x30_udp_protocol_test"
"${prefix}/bin/x30_factory_contract_test"
loader_output="$(ldd "${prefix}/bin/robot_test_x30")"
printf "%s\n" "${loader_output}"
resolved_library="$(
  awk '"'"'$1 == "librobot_hardware_x30.so" {print $3}'"'"' \
    <<<"${loader_output}"
)"
# 要求 robot_test_x30 解析到本镜像安装的 HAL，
# 而不是基础镜像中同名的其他库。
if [[ -z "${resolved_library}" || ! -e "${resolved_library}" ]]; then
  echo "[hal-offline] ERROR: installed HAL library was not resolved." >&2
  exit 1
fi
expected_library="$(readlink -f "${expected_library}")"
resolved_library="$(readlink -f "${resolved_library}")"
if [[ "${resolved_library}" != "${expected_library}" ]]; then
  echo "[hal-offline] ERROR: wrong HAL library: ${resolved_library:-missing}" >&2
  exit 1
fi
'

echo "[hal-offline] X30 HAL image verification passed."
