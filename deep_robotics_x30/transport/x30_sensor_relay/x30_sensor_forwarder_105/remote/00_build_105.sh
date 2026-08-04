#!/bin/bash
set -euo pipefail

# 105 离线构建入口：编译并测试只订阅转发节点，但不启动 ROS 节点。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${ROOT}/catkin_ws"
TOPLEVEL_TEMPLATE="${ROOT}/remote/catkin_workspace_toplevel.cmake"
TOPLEVEL_FILE="${WORKSPACE}/src/CMakeLists.txt"

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

echo "[105-build] workspace: ${WORKSPACE}"
echo "[105-build] building read-only ROS1 sensor forwarder..."

if [[ ! -f "${TOPLEVEL_TEMPLATE}" ]]; then
  echo "[105-build] ERROR: bundled Catkin toplevel template is missing." >&2
  exit 1
fi

# 原厂 Noetic 会创建一个软链接，其中的相对 include(config.cmake) 在独立
# 工作区内失效。因此使用自包含工作区，且绝不通过该软链接写入。
if [[ -L "${TOPLEVEL_FILE}" ]]; then
  rm -f -- "${TOPLEVEL_FILE}"
fi
if [[ ! -f "${TOPLEVEL_FILE}" ]] ||
   ! cmp -s "${TOPLEVEL_TEMPLATE}" "${TOPLEVEL_FILE}"; then
  cp -- "${TOPLEVEL_TEMPLATE}" "${TOPLEVEL_FILE}"
fi

catkin_make \
  -C "${WORKSPACE}" \
  --force-cmake \
  -DCMAKE_BUILD_TYPE=Release

source "${WORKSPACE}/devel/setup.bash"
rospack find x30_sensor_forwarder_ros1

# 在确认迁移工作区可部署前，同时验证运行程序和写入端帧合同。
test -x "${WORKSPACE}/devel/lib/x30_sensor_forwarder_ros1/x30_sensor_forwarder_node"

PROTOCOL_TEST="${WORKSPACE}/build/x30_sensor_forwarder_ros1/x30_sensor_wire_protocol_writer_test"
test -x "${PROTOCOL_TEST}"
"${PROTOCOL_TEST}"

(
  cd "${WORKSPACE}/build"
  ctest \
    --output-on-failure \
    -R "^x30_sensor_wire_protocol_writer_test$"
)

echo "[105-build] build complete."
echo "[105-build] no ROS node was started."
