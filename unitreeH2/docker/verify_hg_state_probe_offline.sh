#!/usr/bin/env bash

# Offline semantic/safety validation for the Stage 05 HG state probe.
# Run inside unitree_h2:amd64-offline with --network none and mount the probe
# directory read-only at /probe. This does not validate the different PC2 SDK.

set -euo pipefail

SOURCE_DIR=/probe
BUILD_DIR=/tmp/hg_state_probe_build
BINARY="${BUILD_DIR}/h2_hg_state_read_only_probe"

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR=/opt/unitree_robotics/lib/cmake/unitree_sdk2
cmake --build "${BUILD_DIR}" --parallel 2

file "${BINARY}"
sha256sum "${BINARY}"
readelf -h "${BINARY}" | grep -E 'Class:|Machine:'

nm -C "${BINARY}" >"${BUILD_DIR}/binary_symbols.txt"
forbidden='CreateSendChannel|ChannelPublisher|unitree::robot::[^ ]*Client|LowCmd_|MotorCmd_|MotorCmds_|HandCmd_|BmsCmd_|Request_|dds_create_writer|dds_write|dds_writedispose'
if grep -nE "${forbidden}" "${BUILD_DIR}/binary_symbols.txt"; then
  echo "OFFLINE_BINARY_SEND_PATH_AUDIT_FAILED"
  exit 20
fi
grep -m1 'CreateRecvChannel' "${BUILD_DIR}/binary_symbols.txt"

dangerous_strings='rt/lowcmd|rt/arm_sdk|rt/api/|/api/|motion_switcher'
if strings "${BINARY}" | grep -nE "${dangerous_strings}"; then
  echo "OFFLINE_BINARY_DANGEROUS_STRING_AUDIT_FAILED"
  exit 21
fi

actual_topics="$(strings "${BINARY}" | grep -E '^rt/' | sort -u || true)"
expected_topics=$'rt/lf/bmsstate\nrt/lf/lowstate\nrt/lf/mainboardstate\nrt/lf/secondary_imu\nrt/lowstate\nrt/lowstate_raw\nrt/secondary_imu'
printf 'BINARY_RT_TOPICS_BEGIN\n%s\nBINARY_RT_TOPICS_END\n' "${actual_topics}"
if [[ "${actual_topics}" != "${expected_topics}" ]]; then
  echo "OFFLINE_BINARY_TOPIC_ALLOWLIST_MISMATCH"
  exit 22
fi

echo "STAGE05_HG_STATE_PROBE_OFFLINE_SAFETY_OK"
