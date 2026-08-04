#!/usr/bin/env bash

# Unitree H2 PC2 SDK2/工具链 ABI 只读审计。
# 用途：记录编译器、glibc、SDK2 静态库、CMake 导入契约和 CycloneDDS
# 动态库的架构/符号版本/哈希，为离线构建包能否在 PC2 运行提供依据。
# 安全边界：不编译、不初始化 DDS、不创建 reader/writer、不调用 RPC、
# 不发布数据，也不修改服务、软件包或网络。

# 单项 ABI 检查失败后继续输出其余证据；最终应检查所有 COMMAND_RC。
set +e

# 统一日志分节。
section() {
  printf '\n===== %s =====\n' "$1"
}

# 执行只读命令并把返回码写成稳定键；始终返回 0 以继续完整审计。
run_check() {
  local label="$1"
  shift
  "$@"
  local rc=$?
  printf 'COMMAND_RC[%s]=%s\n' "$label" "$rc"
  return 0
}

# 交付机 SDK2 的权威候选安装根和静态库位置。
SDK_ROOT=/opt/unitree_robotics
SDK_LIB="${SDK_ROOT}/lib/libunitree_sdk2.a"

section "TIME HOST AND SAFETY SCOPE"
# 记录主机、脚本绝对路径和显式只读声明，防止日志来源混淆。
date --iso-8601=seconds
hostname
uname -a
id
printf 'SCRIPT=%s\n' "$(readlink -f "$0")"
printf 'SDK_ROOT=%s\n' "$SDK_ROOT"
echo "READ_ONLY_ABI_AUDIT_NO_DDS_INIT_NO_BUILD_NO_RPC"

section "TOOLCHAIN AVAILABILITY"
# 先探测工具路径，再记录版本和 dpkg 架构；不安装缺失工具。
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
# 固定清单覆盖 SDK 库、CMake 导入文件、Channel/Client API 与 H2/HG IDL；
# 每项记录存在性、格式和哈希，不读取机器人状态。
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
# 只查找 H2 高层运动客户端头文件，确认开发 API 是否随交付系统安装。
# 找到头文件不代表已连接 PC1，也不触发任何运动调用。
for root in /opt/unitree_robotics /usr/local /unitree/opt /home/unitree; do
  if [[ -d "$root" ]]; then
    run_check "h2_loco_header_search:${root}" timeout 30s find "$root" -type f \
      \( -name 'h2_loco_api.hpp' -o -name 'h2_loco_client.hpp' \) \
      -print 2>/dev/null
  fi
done
echo "H2_LOCO_HEADER_SEARCH_COMPLETE"

section "CMAKE IMPORT CONTRACT"
# 摘取 SDK2 声明版本、include 路径、链接依赖和导入库位置。
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
# 静态分析 libunitree_sdk2.a 的对象格式、相关导出符号及最高运行库版本；
# ar/nm/objdump/strings 均不加载或执行库内代码。
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
# 对两个安装树中的 CycloneDDS 库解析实际文件，并记录 ELF 架构、SONAME、
# 依赖和符号版本，排查容器/主机 ABI 不兼容。
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

# 字节比较两个安装树，确认它们是相同副本而非版本分叉。
run_check "compare_ddsc_trees" cmp -s \
  /unitree/opt/lib/libddsc.so /opt/unitree_robotics/lib/libddsc.so
run_check "compare_ddscxx_trees" cmp -s \
  /unitree/opt/lib/libddscxx.so /opt/unitree_robotics/lib/libddscxx.so

section "LOADER VIEW"
# 记录动态加载器缓存和当前 LD_LIBRARY_PATH，解释运行时实际选库顺序。
ldconfig -p | grep -E 'libunitree_sdk2|libddsc(xx)?\.so' || true
printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH-}"

section "PROCESS AND DDS NON-INTERACTION CHECK"
# 再次声明并检查本脚本未启动 ROS CLI daemon 或额外 DDS/控制进程。
echo "No ChannelFactory::Init, DDS participant, subscriber, publisher or RPC was created."
pgrep -af '04_pc2_sdk2_abi_read_only_audit|ros2cli\.daemon\.daemonize' || true

section "AUDIT COMPLETE"
# 结束标记表示静态审计执行完毕，不等价于实机 RPC 或运动测试通过。
echo "READ_ONLY_PC2_SDK2_ABI_AUDIT_OK"
