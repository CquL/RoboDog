#!/bin/bash
set -euo pipefail

# 构建 106 生产镜像，再在禁用网络的临时容器中验证被动接收端和 X30 HAL。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"

echo "[build] root: ${ROOT}"
echo "[build] image: ${IMAGE}"
docker build -t "${IMAGE}" "${ROOT}"

# 此检查验证 ROS2 包和 launch 文件已安装。容器使用 --network none，
# 且不运行 launch，因此不会打开转发端口。
echo "[build] verifying passive 105-to-106 sensor receiver..."
docker run --rm --network none --entrypoint bash "${IMAGE}" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
ros2 pkg prefix x30_sensor_receiver
test -x /ws/install/x30_sensor_receiver/lib/x30_sensor_receiver/x30_sensor_receiver_node
test -f /ws/install/x30_sensor_receiver/share/x30_sensor_receiver/launch/passive_receiver.launch.py
echo "[build] passive sensor receiver verified."
'

echo "[build] verifying installed X30 robot hardware HAL..."
# 协议/工厂测试及 ldd 校验均离线执行，不初始化 HAL，也不向机器人发送 UDP。
docker run --rm --network none --entrypoint bash "${IMAGE}" -lc '
set -euo pipefail
prefix=/opt/x30_robot_hardware
expected_library="${prefix}/lib/librobot_hardware_x30.so"
test -f "${expected_library}"
test -f "${prefix}/include/robot_hardware/robot_hardware_interface.h"
test -f "${prefix}/include/robot_hardware/deep_robotics/deep_robotics_x30.h"
test -x "${prefix}/bin/robot_test_x30"
test -x "${prefix}/bin/x30_udp_protocol_test"
test -x "${prefix}/bin/x30_factory_contract_test"
test -f "${prefix}/share/robot_hardware/config/config.yaml"
/opt/x30_robot_hardware/bin/x30_udp_protocol_test
/opt/x30_robot_hardware/bin/x30_factory_contract_test
loader_output="$(ldd "${prefix}/bin/robot_test_x30")"
printf "%s\n" "${loader_output}"
resolved_library="$(
  awk '"'"'$1 == "librobot_hardware_x30.so" {print $3}'"'"' \
    <<<"${loader_output}"
)"
if [[ -z "${resolved_library}" || ! -e "${resolved_library}" ]]; then
  echo "[build] ERROR: X30 executable did not resolve the installed HAL library." >&2
  exit 1
fi
expected_library="$(readlink -f "${expected_library}")"
resolved_library="$(readlink -f "${resolved_library}")"
if [[ "${resolved_library}" != "${expected_library}" ]]; then
  echo "[build] ERROR: X30 executable resolved the wrong HAL library: ${resolved_library:-missing}" >&2
  exit 1
fi
echo "[build] X30 HAL and factory contract verified without network access."
'

echo "[build] done"
docker images | grep -E 'REPOSITORY|x30_livox|jezetek' || true
