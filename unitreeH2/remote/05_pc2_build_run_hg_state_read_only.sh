#!/usr/bin/env bash

# Build and run the H2 HG native DDS state probe on PC2.
# The application creates receive channels only and has a bounded runtime.
# SDK2 Init may allocate an internal publisher container, but this application
# never creates a DataWriter/send channel and never invokes a control client.

set -euo pipefail

ROOT=/home/unitree/p2_unitreeH2
SOURCE_DIR="${ROOT}/src/hg_state_probe"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${BASHPID}"
BUILD_DIR="${ROOT}/build/hg_state_probe_stage05_${RUN_ID}"
BINARY="${BUILD_DIR}/h2_hg_state_read_only_probe"
SDK_PREFIX=/opt/unitree_robotics
CYCLONE_CONFIG=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml

section() {
  printf '\n===== %s =====\n' "$1"
}

run_check() {
  local label="$1"
  shift
  local rc=0
  if "$@"; then
    rc=0
  else
    rc=$?
  fi
  printf 'COMMAND_RC[%s]=%s\n' "$label" "$rc"
  return "$rc"
}

verify_sha256() {
  local expected="$1"
  local path="$2"
  printf '%s  %s\n' "$expected" "$path" | sha256sum --check --strict -
}

section "PATH CONTRACT"
printf 'ROOT=%s\nSOURCE_DIR=%s\nBUILD_DIR=%s\n' \
  "${ROOT}" "${SOURCE_DIR}" "${BUILD_DIR}"
mkdir -p "${ROOT}/scripts" "${ROOT}/logs" "${ROOT}/src" \
  "${ROOT}/build" "${ROOT}/config" "${BUILD_DIR}"

section "PRECONDITIONS"
for tool in cmake g++ make file sha256sum ldd timeout nm readelf strings \
  grep sort ip pgrep; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    printf 'MISSING_TOOL=%s\n' "${tool}"
    exit 10
  fi
  command -v "${tool}"
done
cmake --version | head -n 1
g++ --version | head -n 1

if ! ip -o link show dev eth0 | grep -qE '<[^>]*(UP|LOWER_UP)[^>]*>'; then
  echo "ETH0_NOT_UP"
  exit 11
fi
if ! ip -o -4 addr show dev eth0 | grep -q '192\.168\.123\.162/24'; then
  echo "ETH0_EXPECTED_ADDRESS_MISSING"
  exit 11
fi
if [[ ! -r "${CYCLONE_CONFIG}" || ! -f "${CYCLONE_CONFIG}" ]]; then
  printf 'CYCLONEDDS_CONFIG_NOT_READABLE=%s\n' "${CYCLONE_CONFIG}"
  exit 11
fi

required_files=(
  "${SOURCE_DIR}/CMakeLists.txt"
  "${SOURCE_DIR}/h2_hg_state_read_only_probe.cpp"
  "${SDK_PREFIX}/include/unitree/idl/hg/LowState_.hpp"
  "${SDK_PREFIX}/include/unitree/idl/hg/IMUState_.hpp"
  "${SDK_PREFIX}/include/unitree/idl/hg/BmsState_.hpp"
  "${SDK_PREFIX}/include/unitree/idl/hg/MainBoardState_.hpp"
  "${SDK_PREFIX}/include/unitree/robot/channel/channel_factory.hpp"
  "${SDK_PREFIX}/include/unitree/robot/channel/channel_subscriber.hpp"
  "${SDK_PREFIX}/lib/libunitree_sdk2.a"
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2ConfigVersion.cmake"
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Targets.cmake"
  "${SDK_PREFIX}/lib/libddsc.so"
  "${SDK_PREFIX}/lib/libddscxx.so"
  "${SDK_PREFIX}/lib/x86_64/libddsc.so"
  "${SDK_PREFIX}/lib/x86_64/libddscxx.so"
)
for path in "${required_files[@]}"; do
  if [[ ! -f "${path}" ]]; then
    printf 'MISSING_REQUIRED_FILE=%s\n' "${path}"
    exit 11
  fi
done

section "SOURCE SAFETY AUDIT"
forbidden='ChannelPublisher|CreateSendChannel|LowCmd|MotorCmd|HandCmd|BmsCmd|lowcmd|arm_sdk|motion_switcher|SetVelocity|StopMove|LocoClient|Client::|Request_|dds_create_writer|dds_write|ros2[[:space:]]+topic[[:space:]]+pub|ros2[[:space:]]+service[[:space:]]+call'
if grep -R -n -E "${forbidden}" \
  "${SOURCE_DIR}/CMakeLists.txt" \
  "${SOURCE_DIR}/h2_hg_state_read_only_probe.cpp"; then
  echo "SOURCE_SAFETY_AUDIT_FAILED"
  exit 12
fi
echo "SOURCE_SAFETY_AUDIT_OK"

section "PINNED SOURCE SDK AND NETWORK HASHES"
verify_sha256 "0d9d32404ad833acc25eba899ad345899f599ca2ee710a22e0014ea1dd84d492" \
  "${SOURCE_DIR}/CMakeLists.txt"
verify_sha256 "d794406dfbeb4c88c1256bf24dcaba6ae3c5c9b504b10313de0de90a097be844" \
  "${SOURCE_DIR}/h2_hg_state_read_only_probe.cpp"
verify_sha256 "e01be29667100a6f2cd829b3ea8a90f846271b20a0cde96331ceecfb4d46e042" \
  "${SDK_PREFIX}/include/unitree/idl/hg/LowState_.hpp"
verify_sha256 "0b078db5401a35c2421190adb98c0a7e94472ee798b20aa4d1a345513dc88b1b" \
  "${SDK_PREFIX}/include/unitree/idl/hg/IMUState_.hpp"
verify_sha256 "1d1a83921b7db50a26523574093e127df54c9bedf6ee782474f734d6a86f23ba" \
  "${SDK_PREFIX}/include/unitree/idl/hg/BmsState_.hpp"
verify_sha256 "5ec95395a96da501d627aeca6db9d695cf0e50c6da6de670ff5371c4d2dbc9df" \
  "${SDK_PREFIX}/include/unitree/idl/hg/MainBoardState_.hpp"
verify_sha256 "e0568b06205599de89543c8c11459660ecf9dc6203bf43a778b25afaf5a2d529" \
  "${SDK_PREFIX}/include/unitree/robot/channel/channel_factory.hpp"
verify_sha256 "8fbd4fa5f4f9f6ffd7c34f2500f9daa8304ec7ae5c15f21e2e6ff67a9cbb6ee0" \
  "${SDK_PREFIX}/include/unitree/robot/channel/channel_subscriber.hpp"
verify_sha256 "e436eedf0d81e9efa10b039f8151743f46547535a99790ed19ddacc105098cd4" \
  "${SDK_PREFIX}/lib/libunitree_sdk2.a"
verify_sha256 "39f73d1cb56f9d1fd383e1b2c47d30a2bd730e83cc2a49a401c8de5e81b54709" \
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
verify_sha256 "7fa49a7a57dff5f7f89355693ca51982fab0009a983b568860f823b0d44bfc45" \
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2ConfigVersion.cmake"
verify_sha256 "7c74357c2f47be5d37fbca8bc476e5062503dfd6c4f63274ce43f362e26e2234" \
  "${SDK_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Targets.cmake"
verify_sha256 "4038630c231412f7b34a2ea60df192bbdebd0a57f22ac7f35c1b6d28323e695c" \
  "${SDK_PREFIX}/lib/libddsc.so"
verify_sha256 "d3e7c1b03123c2745839f2465041777ded090ad66d62f1372949209254f7ebe5" \
  "${SDK_PREFIX}/lib/libddscxx.so"
verify_sha256 "4038630c231412f7b34a2ea60df192bbdebd0a57f22ac7f35c1b6d28323e695c" \
  "${SDK_PREFIX}/lib/x86_64/libddsc.so"
verify_sha256 "d3e7c1b03123c2745839f2465041777ded090ad66d62f1372949209254f7ebe5" \
  "${SDK_PREFIX}/lib/x86_64/libddscxx.so"
verify_sha256 "bc977aacd0e44804cb8da24d24d6fe5ed654aad9ed7b15ed3ef36d32f27a1796" \
  "${CYCLONE_CONFIG}"
echo "PINNED_HASH_CONTRACT_OK"
echo "FRESH_BUILD_DIRECTORY=${BUILD_DIR}"

section "CONFIGURE AND BUILD"
run_check "cmake_configure" cmake \
  -S "${SOURCE_DIR}" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR="${SDK_PREFIX}/lib/cmake/unitree_sdk2" \
  -DCMAKE_BUILD_RPATH="${SDK_PREFIX}/lib" || exit $?
run_check "cmake_build" cmake --build "${BUILD_DIR}" --parallel 2 || exit $?

export LD_LIBRARY_PATH="${SDK_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

section "BINARY DEPENDENCY AND SEND-PATH AUDIT"
file "${BINARY}"
sha256sum "${BINARY}"
readelf -h "${BINARY}" | grep -E 'Class:|Machine:'
readelf -d "${BINARY}" | grep -F "${SDK_PREFIX}/lib"
ldd_output="$(ldd "${BINARY}")"
printf '%s\n' "${ldd_output}"
if grep -q 'not found' <<<"${ldd_output}"; then
  echo "BINARY_DEPENDENCY_NOT_FOUND"
  exit 14
fi
if ! grep -Eq 'libddscxx\.so\.0 => /opt/unitree_robotics/lib/(x86_64/)?libddscxx\.so\.0' \
    <<<"${ldd_output}" || \
   ! grep -Eq 'libddsc\.so\.0 => /opt/unitree_robotics/lib/(x86_64/)?libddsc\.so\.0' \
    <<<"${ldd_output}"; then
  echo "BINARY_DDS_LIBRARY_PATH_MISMATCH"
  exit 14
fi

nm -C "${BINARY}" >"${BUILD_DIR}/binary_symbols.txt"
binary_forbidden='CreateSendChannel|ChannelPublisher|unitree::robot::[^ ]*Client|LowCmd_|MotorCmd_|MotorCmds_|HandCmd_|BmsCmd_|Request_|dds_create_writer|dds_write|dds_writedispose'
if grep -nE "${binary_forbidden}" "${BUILD_DIR}/binary_symbols.txt"; then
  echo "BINARY_SEND_PATH_AUDIT_FAILED"
  exit 15
fi
if ! grep -q 'CreateRecvChannel' "${BUILD_DIR}/binary_symbols.txt"; then
  echo "BINARY_RECEIVE_PATH_SYMBOL_MISSING"
  exit 15
fi

dangerous_strings='rt/lowcmd|rt/arm_sdk|rt/api/|/api/|motion_switcher'
if strings "${BINARY}" | grep -nE "${dangerous_strings}"; then
  echo "BINARY_DANGEROUS_STRING_AUDIT_FAILED"
  exit 15
fi
actual_topics="$(strings "${BINARY}" | grep -E '^rt/' | sort -u || true)"
expected_topics=$'rt/lf/bmsstate\nrt/lf/lowstate\nrt/lf/mainboardstate\nrt/lf/secondary_imu\nrt/lowstate\nrt/lowstate_raw\nrt/secondary_imu'
printf 'BINARY_RT_TOPICS_BEGIN\n%s\nBINARY_RT_TOPICS_END\n' "${actual_topics}"
if [[ "${actual_topics}" != "${expected_topics}" ]]; then
  echo "BINARY_TOPIC_ALLOWLIST_MISMATCH"
  exit 15
fi
echo "BINARY_RECEIVE_ONLY_AUDIT_OK"

section "BOUNDED SUBSCRIPTION RUN"
export CYCLONEDDS_URI="${CYCLONE_CONFIG}"
if pgrep -af '[h]2_hg_state_read_only_probe'; then
  echo "PREEXISTING_H2_HG_PROBE_REFUSED"
  exit 16
fi
set +e
timeout --signal=INT --kill-after=5s 25s "${BINARY}" \
  --interface eth0 \
  --domain 0 \
  --seconds 15
probe_rc=$?
set -e
printf 'COMMAND_RC[h2_hg_state_read_only_probe]=%s\n' "${probe_rc}"

section "PROCESS CLEANUP CHECK"
if pgrep -af '[h]2_hg_state_read_only_probe'; then
  echo "H2_HG_PROBE_PROCESS_STILL_RUNNING"
  exit 17
fi
echo "H2_HG_PROBE_NOT_RUNNING"

if [[ "${probe_rc}" -ne 0 ]]; then
  echo "H2_HG_STATE_PROBE_RUN_FAILED_OR_INCOMPLETE"
  exit "${probe_rc}"
fi

echo "READ_ONLY_PC2_HG_STATE_PROBE_OK"
