#!/usr/bin/env bash

# Unitree H2 PC2 first-pass inventory.
# This script is intentionally read-only: no sudo, package installation,
# service restart, network reconfiguration, Docker launch, or control command.

set +e

section() {
  printf '\n===== %s =====\n' "$1"
}

section "TIME AND HOST"
date --iso-8601=seconds
hostnamectl
uname -a
uname -m

section "OPERATING SYSTEM"
cat /etc/os-release
cat /proc/cmdline
timedatectl

section "CPU MEMORY STORAGE"
lscpu
free -h
lsblk -o NAME,TYPE,SIZE,FSTYPE,MOUNTPOINTS,MODEL,SERIAL
df -hT

section "NETWORK"
ip -br link
ip -br address
ip route
ip rule
ip neigh

section "PCI USB AND USER DEVICES"
lspci -nn
lsusb
find /dev -maxdepth 1 \( -name 'video*' -o -name 'ttyUSB*' -o -name 'ttyACM*' -o -name 'can*' \) -printf '%f\n' | sort

section "GPU"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi
else
  echo "nvidia-smi: not installed"
fi

section "DOCKER"
if command -v docker >/dev/null 2>&1; then
  docker version
  docker compose version
  docker ps --no-trunc
  docker images --digests
else
  echo "docker: not installed"
fi

section "ROS DDS ENVIRONMENT"
command -v ros2
command -v roscore
find /opt/ros -maxdepth 2 -type f -name setup.bash -print 2>/dev/null
env | grep -E '^(ROS|RMW|CYCLONE|FASTRTPS|AMENT|COLCON|UNITREE)' | sort

section "RUNNING SERVICES"
systemctl --failed --no-pager
systemctl list-units --type=service --state=running --no-pager | grep -Ei 'unitree|sport|loco|motion|dds|cyclone|ros|camera|lidar|imu|docker|container'

section "RELEVANT PROCESSES"
ps -eo user,pid,ppid,rtprio,ni,stat,lstart,args --sort=pid | grep -Ei 'unitree|sport|loco|motion|dds|cyclone|ros|camera|lidar|imu|docker|containerd' | grep -v grep

section "LISTENING SOCKETS"
ss -lntup

section "INSTALLED ROBOTICS PACKAGES"
dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null | grep -Ei 'unitree|ros-|cyclone|fastdds|rmw|docker|nvidia|cuda|realtime'

section "LIKELY UNITREE AND ROS WORKSPACES"
for root in /home/unitree /opt /usr/local; do
  if [[ -d "${root}" ]]; then
    find "${root}" -maxdepth 4 -type d \
      \( -iname '*unitree*' -o -iname '*sdk2*' -o -iname '*ros2*' \
         -o -iname '*cyclone*' -o -iname '*loco*' -o -iname '*sport*' \) \
      -print 2>/dev/null
  fi
done

section "INVENTORY COMPLETE"
echo "READ_ONLY_PC2_INVENTORY_OK"
