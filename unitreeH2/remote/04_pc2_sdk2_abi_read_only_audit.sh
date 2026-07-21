#!/usr/bin/env bash

# Unitree H2 PC2 SDK2/toolchain ABI audit only.
# This script reads installed files and binary metadata. It does not compile,
# initialize DDS, create participants/readers/writers, call RPC, publish data,
# alter services, install packages or change network configuration.

set +e

section() {
  printf '\n===== %s =====\n' "$1"
}

run_check() {
  local label="$1"
  shift
  "$@"
  local rc=$?
  printf 'COMMAND_RC[%s]=%s\n' "$label" "$rc"
  return 0
}

SDK_ROOT=/opt/unitree_robotics
SDK_LIB="${SDK_ROOT}/lib/libunitree_sdk2.a"

section "TIME HOST AND SAFETY SCOPE"
date --iso-8601=seconds
hostname
uname -a
id
printf 'SCRIPT=%s\n' "$(readlink -f "$0")"
printf 'SDK_ROOT=%s\n' "$SDK_ROOT"
echo "READ_ONLY_ABI_AUDIT_NO_DDS_INIT_NO_BUILD_NO_RPC"

section "TOOLCHAIN AVAILABILITY"
for tool in cmake g++ gcc ld ar nm readelf objdump strings sha256sum file; do
  printf '\n--- %s ---\n' "$tool"
  command -v "$tool" || true
done
run_check "cmake_version" cmake --version
run_check "gxx_version" g++ --version
run_check "gcc_version" gcc --version
run_check "ld_version" ld --version
run_check "libc_version" ldd --version
run_check "package_versions" dpkg-query -W \
  -f='${binary:Package}\t${Version}\t${Architecture}\n' \
  cmake g++ gcc binutils libc6 libstdc++6

section "SDK2 INSTALLATION FILES"
artifacts=(
  "${SDK_ROOT}/lib/libunitree_sdk2.a"
  "${SDK_ROOT}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
  "${SDK_ROOT}/lib/cmake/unitree_sdk2/unitree_sdk2ConfigVersion.cmake"
  "${SDK_ROOT}/lib/cmake/unitree_sdk2/unitree_sdk2Targets.cmake"
  "${SDK_ROOT}/include/unitree/robot/channel/channel_factory.hpp"
  "${SDK_ROOT}/include/unitree/robot/channel/channel_subscriber.hpp"
  "${SDK_ROOT}/include/unitree/robot/client/client.hpp"
  "${SDK_ROOT}/include/unitree/robot/client/client_base.hpp"
  "${SDK_ROOT}/include/unitree/robot/client/client_stub.hpp"
  "${SDK_ROOT}/include/unitree/idl/hg/LowState_.hpp"
  "${SDK_ROOT}/include/unitree/idl/hg/IMUState_.hpp"
  "${SDK_ROOT}/include/unitree/idl/hg/BmsState_.hpp"
  "${SDK_ROOT}/include/unitree/idl/hg/MainBoardState_.hpp"
  "${SDK_ROOT}/include/unitree/idl/hg/SportModeState_.hpp"
)
for artifact in "${artifacts[@]}"; do
  printf '\n--- %s ---\n' "$artifact"
  if [[ -e "$artifact" || -L "$artifact" ]]; then
    ls -l "$artifact"
    file -L "$artifact"
    sha256sum "$artifact"
  else
    echo "MISSING"
  fi
done

section "H2 LOCO HEADER PRESENCE"
for root in /opt/unitree_robotics /usr/local /unitree/opt /home/unitree; do
  if [[ -d "$root" ]]; then
    run_check "h2_loco_header_search:${root}" timeout 30s find "$root" -type f \
      \( -name 'h2_loco_api.hpp' -o -name 'h2_loco_client.hpp' \) \
      -print 2>/dev/null
  fi
done
echo "H2_LOCO_HEADER_SEARCH_COMPLETE"

section "CMAKE IMPORT CONTRACT"
for cmake_file in \
  "${SDK_ROOT}/lib/cmake/unitree_sdk2/unitree_sdk2ConfigVersion.cmake" \
  "${SDK_ROOT}/lib/cmake/unitree_sdk2/unitree_sdk2Targets.cmake"; do
  if [[ -f "$cmake_file" ]]; then
    printf '\n--- %s ---\n' "$cmake_file"
    grep -nE 'PACKAGE_VERSION|add_library|IMPORTED_LOCATION|INTERFACE_INCLUDE_DIRECTORIES|INTERFACE_LINK_LIBRARIES|LINKER_LANGUAGE' \
      "$cmake_file"
  fi
done

section "SDK2 ARCHIVE STRUCTURE"
if [[ -f "$SDK_LIB" ]]; then
  run_check "archive_member_count" bash -c 'ar t "$1" | wc -l' _ "$SDK_LIB"
  echo "--- selected archive members ---"
  ar t "$SDK_LIB" | grep -Ei 'channel|client|loco|rpc|dds|factory' | sed -n '1,400p'
  echo "--- unique object formats ---"
  timeout 30s objdump -f "$SDK_LIB" 2>/dev/null \
    | grep -E 'file format|architecture:' \
    | sort -u
  echo "--- selected exported symbols ---"
  timeout 45s nm -C -g --defined-only "$SDK_LIB" 2>/dev/null \
    | grep -E 'ChannelFactory|Client::|ClientStub|DdsFactory|CreateRecvChannel|CreateSendChannel' \
    | sed -n '1,500p'
  echo "--- highest referenced runtime symbol versions ---"
  strings "$SDK_LIB" | grep -oE 'GLIBCXX_[0-9.]+' | sort -Vu | tail -n 20
  strings "$SDK_LIB" | grep -oE 'CXXABI_[0-9.]+' | sort -Vu | tail -n 20
  strings "$SDK_LIB" | grep -oE 'GLIBC_[0-9.]+' | sort -Vu | tail -n 20
else
  echo "SDK2_ARCHIVE_MISSING"
fi

section "DDS LIBRARY RESOLUTION AND ABI"
for lib in \
  /unitree/opt/lib/libddsc.so.0 \
  /unitree/opt/lib/libddscxx.so.0 \
  /opt/unitree_robotics/lib/libddsc.so.0 \
  /opt/unitree_robotics/lib/libddscxx.so.0; do
  printf '\n--- %s ---\n' "$lib"
  if [[ -e "$lib" || -L "$lib" ]]; then
    resolved="$(readlink -f "$lib")"
    ls -l "$lib"
    printf 'resolved=%s\n' "$resolved"
    sha256sum "$resolved"
    readelf -h "$resolved" | grep -E 'Class:|Data:|Machine:|OS/ABI:'
    readelf -d "$resolved" | grep -E 'SONAME|NEEDED'
    strings "$resolved" | grep -oE 'GLIBCXX_[0-9.]+' | sort -Vu | tail -n 10
    strings "$resolved" | grep -oE 'CXXABI_[0-9.]+' | sort -Vu | tail -n 10
    strings "$resolved" | grep -oE 'GLIBC_[0-9.]+' | sort -Vu | tail -n 10
  else
    echo "MISSING"
  fi
done

run_check "compare_ddsc_trees" cmp -s \
  /unitree/opt/lib/libddsc.so /opt/unitree_robotics/lib/libddsc.so
run_check "compare_ddscxx_trees" cmp -s \
  /unitree/opt/lib/libddscxx.so /opt/unitree_robotics/lib/libddscxx.so

section "LOADER VIEW"
ldconfig -p | grep -E 'libunitree_sdk2|libddsc(xx)?\.so' || true
printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH-}"

section "PROCESS AND DDS NON-INTERACTION CHECK"
echo "No ChannelFactory::Init, DDS participant, subscriber, publisher or RPC was created."
pgrep -af '04_pc2_sdk2_abi_read_only_audit|ros2cli\.daemon\.daemonize' || true

section "AUDIT COMPLETE"
echo "READ_ONLY_PC2_SDK2_ABI_AUDIT_OK"
