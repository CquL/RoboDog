#!/usr/bin/env bash

# Unitree H2 PC2 第二轮 DDS/SDK2 只读审计。
# 用途：确认 eth0/eth1 的物理映射、原厂 CycloneDDS 配置、相关进程环境、
# SDK2 安装位置/库解析和容器运行时现状。输出由调用者用 tee 保存为日志。
# 安全边界：不使用 sudo、不改软件包/服务/网络、不启动容器，不创建 DDS
# participant，也不调用 H2 控制 RPC。

# 某项缺失不应中断后续证据采集；各命令的文本结果需分别解释。
set +e

# 输出稳定的日志分节标题。
section() {
  printf '\n===== %s =====\n' "$1"
}

section "TIME AND HOST"
# 标记日志来源和生成时间。
date --iso-8601=seconds
hostname

section "NETWORK DEVICE MAPPING"
# 分别核对两张原厂网卡的总线、驱动、udev 属性、地址与组播成员。
# 所有命令均为查询操作，不改变链路状态。
for nic in eth0 eth1; do
  printf '\n--- %s ---\n' "${nic}"
  readlink -f "/sys/class/net/${nic}/device" 2>/dev/null
  if command -v ethtool >/dev/null 2>&1; then
    ethtool -i "${nic}" 2>/dev/null
  else
    echo "ethtool: not installed"
  fi
  udevadm info -q property -p "/sys/class/net/${nic}" 2>/dev/null
  ip -details link show dev "${nic}"
  ip address show dev "${nic}"
  ip maddr show dev "${nic}"
done

section "CYCLONEDDS CONFIGURATION"
# 读取原厂 SLAM/运动进程使用的 CycloneDDS XML，并记录哈希以识别配置漂移。
dds_xml="/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml"
if [[ -f "${dds_xml}" ]]; then
  ls -l "${dds_xml}"
  sha256sum "${dds_xml}"
  sed -n '1,320p' "${dds_xml}"
else
  echo "Missing: ${dds_xml}"
fi

section "RELEVANT PROCESS DDS ENVIRONMENT"
# 从 /proc 读取既有原厂进程的命令行和 DDS/ROS 环境变量；不向进程发信号。
for pattern in unitree_slam dog_cmd sport_switch bms_ctr; do
  printf '\n--- %s ---\n' "${pattern}"
  pids="$(pgrep -f "${pattern}" 2>/dev/null)"
  if [[ -z "${pids}" ]]; then
    echo "not running"
    continue
  fi
  for pid in ${pids}; do
    printf 'pid=%s cmd=' "${pid}"
    tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null
    printf '\n'
    tr '\0' '\n' < "/proc/${pid}/environ" 2>/dev/null |
      grep -E '^(ROS|RMW|CYCLONE|FASTRTPS|AMENT|COLCON|UNITREE)' | sort
  done
done

section "UNITREE AND FAILED SERVICES"
# 读取服务单元定义、当前状态与本次启动日志，用来判断原厂服务所有权。
systemctl status --no-pager --full unitree_slam.service shoujuan_server.service
systemctl cat unitree_slam.service shoujuan_server.service
journalctl -b --no-pager -n 200 \
  -u unitree_slam.service -u shoujuan_server.service

section "SDK2 CMAKE PACKAGES"
# 审计两个常见 SDK2 CMake 安装位置，记录导入目标和声明版本。
for dir in /opt/unitree_robotics/lib/cmake/unitree_sdk2 \
           /usr/local/lib/cmake/unitree_sdk2; do
  printf '\n--- %s ---\n' "${dir}"
  if [[ -d "${dir}" ]]; then
    find "${dir}" -maxdepth 2 \( -type f -o -type l \) \
      -printf '%y %p -> %l\n' | sort
    grep -RInE 'PACKAGE_VERSION|VERSION|unitree_sdk2' "${dir}" 2>/dev/null |
      head -n 240
  else
    echo "not present"
  fi
done

section "SDK2 INSTALLED FILES"
# 列出 /opt 下 SDK2 的头文件、库和符号链接；不复制或覆盖厂商文件。
if [[ -d /opt/unitree_robotics ]]; then
  find /opt/unitree_robotics -maxdepth 4 \( -type f -o -type l \) \
    -printf '%y %p -> %l\n' | sort
else
  echo "/opt/unitree_robotics: not present"
fi

section "SDK2 AND DDS LIBRARIES"
# 查询动态加载器视图及 SDK2/CycloneDDS 实际链接目标，辅助 ABI 判断。
ldconfig -p 2>/dev/null | grep -Ei 'unitree|ddsc|cyclone'
for lib in /opt/unitree_robotics/lib/libunitree_sdk2.so \
           /opt/unitree_robotics/lib/libddsc.so \
           /opt/unitree_robotics/lib/libddsc.so.0 \
           /opt/unitree_robotics/lib/libddscxx.so \
           /opt/unitree_robotics/lib/libddscxx.so.0; do
  if [[ -e "${lib}" || -L "${lib}" ]]; then
    ls -l "${lib}"
    file "${lib}"
    readlink -f "${lib}"
  fi
done

section "CONTAINER RUNTIME COMMANDS"
# 仅探测已安装的容器命令和包版本，不启动 daemon 或容器。
for command_name in docker podman nerdctl ctr containerd; do
  printf '%-12s ' "${command_name}"
  command -v "${command_name}" || true
done
dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null |
  grep -Ei 'docker|podman|containerd|nerdctl|runc'

section "AUDIT COMPLETE"
# 稳定结束标记；不代表 SDK 已经成功连接实机或运动接口已可用。
echo "READ_ONLY_PC2_DDS_SDK_AUDIT_OK"
