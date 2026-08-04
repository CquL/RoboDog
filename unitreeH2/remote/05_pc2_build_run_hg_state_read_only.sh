#!/usr/bin/env bash

# 在 PC2 上构建并运行 H2 HG 原生 DDS 状态只读探针。
# 用途：验证交付机 SDK2/编译器/动态库 ABI，并在 15 秒窗口内被动统计
# LowState、secondary IMU、BMS 和 MainBoard 状态，日志由调用者用 tee 保存。
# 边界：应用只创建 receive channel，运行时间有上限。SDK2 Init 内部可能
# 分配 publisher 容器，但本应用不创建 DataWriter/send channel，也不实例化
# 控制客户端或调用任何运动 RPC。

# 前置条件、哈希、构建、二进制审计或运行任一步失败都立即退出。
set -euo pipefail

# 固定工作区、源码、SDK 和原厂 CycloneDDS 配置；每次使用唯一构建目录，
# 防止复用旧二进制或污染其他 PC2 工件。
ROOT=/home/unitree/p2_unitreeH2
SOURCE_DIR="${ROOT}/src/hg_state_probe"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${BASHPID}"
BUILD_DIR="${ROOT}/build/hg_state_probe_stage05_${RUN_ID}"
BINARY="${BUILD_DIR}/h2_hg_state_read_only_probe"
SDK_PREFIX=/opt/unitree_robotics
CYCLONE_CONFIG=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml

# 稳定日志分节。
section() {
  printf '\n===== %s =====\n' "$1"
}

# 保留被包装命令的真实返回码并输出稳定键；调用者决定是否用 || exit 终止。
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

# 以 sha256sum 严格模式核对单个文件，任何内容漂移都会阻止继续。
verify_sha256() {
  local expected="$1"
  local path="$2"
  printf '%s  %s\n' "$expected" "$path" | sha256sum --check --strict -
}

section "PATH CONTRACT"
# 只在本项目根内创建标准子目录和本次全新构建目录。
printf 'ROOT=%s\nSOURCE_DIR=%s\nBUILD_DIR=%s\n' \
  "${ROOT}" "${SOURCE_DIR}" "${BUILD_DIR}"
mkdir -p "${ROOT}/scripts" "${ROOT}/logs" "${ROOT}/src" \
  "${ROOT}/build" "${ROOT}/config" "${BUILD_DIR}"

section "PRECONDITIONS"
# 工具必须已安装；脚本不会联网或安装缺失软件。
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

# PC2 DDS 网卡必须是 UP 且仍为交付地址，避免绑定错误接口。
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

# 构建只允许使用这组明确的源码、HG IDL、Channel 头、SDK2 和 DDS 库。
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
# 在编译前拒绝发送通道、低层命令、LocoClient、ROS 发布/服务调用等关键词；
# 这是文本门禁，后面还会对最终 ELF 做符号与字符串门禁。
forbidden='ChannelPublisher|CreateSendChannel|LowCmd|MotorCmd|HandCmd|BmsCmd|lowcmd|arm_sdk|motion_switcher|SetVelocity|StopMove|LocoClient|Client::|Request_|dds_create_writer|dds_write|ros2[[:space:]]+topic[[:space:]]+pub|ros2[[:space:]]+service[[:space:]]+call'
if grep -R -n -E "${forbidden}" \
  "${SOURCE_DIR}/CMakeLists.txt" \
  "${SOURCE_DIR}/h2_hg_state_read_only_probe.cpp"; then
  echo "SOURCE_SAFETY_AUDIT_FAILED"
  exit 12
fi
echo "SOURCE_SAFETY_AUDIT_OK"

section "PINNED SOURCE SDK AND NETWORK HASHES"
# 固定源码、SDK/IDL、DDS 库和 CycloneDDS XML 的内容哈希，保证本次验证
# 与已审阅基线完全一致；给源码补注释后必须同步审核对应源码哈希。
verify_sha256 "5e6f75259c50f100a29f43f0f50c42301ed01e77521f8010dc8c67d44d03a19a" \
  "${SOURCE_DIR}/CMakeLists.txt"
verify_sha256 "c6f8f6183424efd137959a811f59c4f83969dd9f4e58872cdd4f34d173694c7f" \
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
# Release 构建显式指向交付 SDK2，并把其 lib 路径写入构建 RPATH。
run_check "cmake_configure" cmake \
  -S "${SOURCE_DIR}" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR="${SDK_PREFIX}/lib/cmake/unitree_sdk2" \
  -DCMAKE_BUILD_RPATH="${SDK_PREFIX}/lib" || exit $?
run_check "cmake_build" cmake --build "${BUILD_DIR}" --parallel 2 || exit $?

# 运行时优先解析同一 SDK 根中的 CycloneDDS 库。
export LD_LIBRARY_PATH="${SDK_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

section "BINARY DEPENDENCY AND SEND-PATH AUDIT"
# 记录 ELF 架构/哈希/依赖；所有 DDS 库必须解析到审核过的 SDK 根。
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

# 保存完整符号清单到本次构建目录，并拒绝任何发送/控制相关符号。
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

# 二进制字符串中也不得出现控制 API/话题；实际 rt/ 话题必须精确匹配
# 下面的七项状态只读白名单。
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
# 使用交付 CycloneDDS XML；若已有同名探针则拒绝并行运行，避免样本归属混淆。
export CYCLONEDDS_URI="${CYCLONE_CONFIG}"
if pgrep -af '[h]2_hg_state_read_only_probe'; then
  echo "PREEXISTING_H2_HG_PROBE_REFUSED"
  exit 16
fi
# 外层 25 秒超时包住应用自身 15 秒窗口；INT 后最多再等 5 秒强制回收。
set +e
timeout --signal=INT --kill-after=5s 25s "${BINARY}" \
  --interface eth0 \
  --domain 0 \
  --seconds 15
probe_rc=$?
set -e
printf 'COMMAND_RC[h2_hg_state_read_only_probe]=%s\n' "${probe_rc}"

section "PROCESS CLEANUP CHECK"
# 实机读取结束后不得残留探针/DDS participant。
if pgrep -af '[h]2_hg_state_read_only_probe'; then
  echo "H2_HG_PROBE_PROCESS_STILL_RUNNING"
  exit 17
fi
echo "H2_HG_PROBE_NOT_RUNNING"

if [[ "${probe_rc}" -ne 0 ]]; then
  echo "H2_HG_STATE_PROBE_RUN_FAILED_OR_INCOMPLETE"
  exit "${probe_rc}"
fi

# 只有应用收到全部要求的状态类别且正常退出，才发布最终成功标记。
echo "READ_ONLY_PC2_HG_STATE_PROBE_OK"
