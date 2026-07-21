#!/usr/bin/env bash

# Unitree H2 PC2 second-pass host, DDS configuration and SDK2 inventory.
# Read-only with respect to the robot and host configuration: no sudo, package
# changes, service restart, network reconfiguration, Docker launch or control RPC.

set +e

section() {
  printf '\n===== %s =====\n' "$1"
}

section "TIME AND HOST"
date --iso-8601=seconds
hostname

section "NETWORK DEVICE MAPPING"
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
dds_xml="/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml"
if [[ -f "${dds_xml}" ]]; then
  ls -l "${dds_xml}"
  sha256sum "${dds_xml}"
  sed -n '1,320p' "${dds_xml}"
else
  echo "Missing: ${dds_xml}"
fi

section "RELEVANT PROCESS DDS ENVIRONMENT"
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
systemctl status --no-pager --full unitree_slam.service shoujuan_server.service
systemctl cat unitree_slam.service shoujuan_server.service
journalctl -b --no-pager -n 200 \
  -u unitree_slam.service -u shoujuan_server.service

section "SDK2 CMAKE PACKAGES"
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
if [[ -d /opt/unitree_robotics ]]; then
  find /opt/unitree_robotics -maxdepth 4 \( -type f -o -type l \) \
    -printf '%y %p -> %l\n' | sort
else
  echo "/opt/unitree_robotics: not present"
fi

section "SDK2 AND DDS LIBRARIES"
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
for command_name in docker podman nerdctl ctr containerd; do
  printf '%-12s ' "${command_name}"
  command -v "${command_name}" || true
done
dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null |
  grep -Ei 'docker|podman|containerd|nerdctl|runc'

section "AUDIT COMPLETE"
echo "READ_ONLY_PC2_DDS_SDK_AUDIT_OK"
