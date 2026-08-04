#!/usr/bin/env bash

# 用途：离线编译并审计 Stage 05 HG 状态探针是否严格保持只读。
# 输入：只读挂载到 /probe 的探针源码和镜像内固定 SDK2。
# 输出：临时 ELF、符号/话题审计结果及 STAGE05_*_OK 标记。
# 安全边界：应在 --network none 容器内运行；脚本拒绝任何发送通道、写 DDS、
#           控制 RPC 字符串或白名单之外的 rt/ 话题。它不等于 PC2 实机验证。

set -euo pipefail

# 所有构建产物只写入 /tmp，不修改只读源码挂载。
SOURCE_DIR=/probe
BUILD_DIR=/tmp/hg_state_probe_build
BINARY="${BUILD_DIR}/h2_hg_state_read_only_probe"

# 编译后先确认 ELF 架构并记录哈希，供 ABI/交付审计。
cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR=/opt/unitree_robotics/lib/cmake/unitree_sdk2
cmake --build "${BUILD_DIR}" --parallel 2

file "${BINARY}"
sha256sum "${BINARY}"
readelf -h "${BINARY}" | grep -E 'Class:|Machine:'

# 符号级拒绝列表覆盖发布器、客户端、低层命令和 DDS writer。
nm -C "${BINARY}" >"${BUILD_DIR}/binary_symbols.txt"
forbidden='CreateSendChannel|ChannelPublisher|unitree::robot::[^ ]*Client|LowCmd_|MotorCmd_|MotorCmds_|HandCmd_|BmsCmd_|Request_|dds_create_writer|dds_write|dds_writedispose'
if grep -nE "${forbidden}" "${BUILD_DIR}/binary_symbols.txt"; then
  echo "OFFLINE_BINARY_SEND_PATH_AUDIT_FAILED"
  exit 20
fi
grep -m1 'CreateRecvChannel' "${BUILD_DIR}/binary_symbols.txt"

# 再用字符串扫描阻止控制话题/API 被间接带入二进制。
dangerous_strings='rt/lowcmd|rt/arm_sdk|rt/api/|/api/|motion_switcher'
if strings "${BINARY}" | grep -nE "${dangerous_strings}"; then
  echo "OFFLINE_BINARY_DANGEROUS_STRING_AUDIT_FAILED"
  exit 21
fi

# 实际 DDS 话题必须与已审计的状态输入白名单逐字一致。
actual_topics="$(strings "${BINARY}" | grep -E '^rt/' | sort -u || true)"
expected_topics=$'rt/lf/bmsstate\nrt/lf/lowstate\nrt/lf/mainboardstate\nrt/lf/secondary_imu\nrt/lowstate\nrt/lowstate_raw\nrt/secondary_imu'
printf 'BINARY_RT_TOPICS_BEGIN\n%s\nBINARY_RT_TOPICS_END\n' "${actual_topics}"
if [[ "${actual_topics}" != "${expected_topics}" ]]; then
  echo "OFFLINE_BINARY_TOPIC_ALLOWLIST_MISMATCH"
  exit 22
fi

echo "STAGE05_HG_STATE_PROBE_OFFLINE_SAFETY_OK"
