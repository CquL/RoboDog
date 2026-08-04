#!/usr/bin/env bash

# Unitree H2 PC2 首轮主机清点脚本。
# 用途：把系统、网络、进程、ROS/DDS 和已安装软件的事实输出到标准输出，
# 调用者可用 tee 保存为带时间戳的日志，作为后续接入工作的只读基线。
# 安全边界：不使用 sudo，不安装软件，不重启服务，不更改网络，不启动
# Docker，也不创建 DDS 写通道或发送任何机器人控制命令。

# 单项检查失败时继续收集其余证据；最终 OK 仅表示脚本走到结尾，
# 不能替代对各节输出的人工/自动验收。
set +e

# 为日志添加稳定分节标题，便于 grep 和跨次审计比较。
section() {
  printf '\n===== %s =====\n' "$1"
}

section "TIME AND HOST"
# 记录时间、主机名、内核和架构，防止把另一台 PC2 的日志误认为本机结果。
date --iso-8601=seconds
hostnamectl
uname -a
uname -m

section "OPERATING SYSTEM"
# 操作系统版本、内核启动参数和时区决定 SDK/容器 ABI 与日志时间解释。
cat /etc/os-release
cat /proc/cmdline
timedatectl

section "CPU MEMORY STORAGE"
# 只查询算力、内存和挂载空间，不执行磁盘写入或性能压力测试。
lscpu
free -h
lsblk -o NAME,TYPE,SIZE,FSTYPE,MOUNTPOINTS,MODEL,SERIAL
df -hT

section "NETWORK"
# 保存接口、地址、路由、策略路由和邻居表；不拉起/关闭任何网络设备。
ip -br link
ip -br address
ip route
ip rule
ip neigh

section "PCI USB AND USER DEVICES"
# 枚举总线及常见相机/串口/CAN 设备节点。设备存在只证明系统识别，
# 不证明传感器正在发布数据。
lspci -nn
lsusb
find /dev -maxdepth 1 \( -name 'video*' -o -name 'ttyUSB*' -o -name 'ttyACM*' -o -name 'can*' \) -printf '%f\n' | sort

section "GPU"
# 若安装 NVIDIA 工具，仅读取驱动/GPU 状态；未安装也保持审计继续。
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi
else
  echo "nvidia-smi: not installed"
fi

section "DOCKER"
# 区分“运行时已安装”和“没有 Docker”；只列容器/镜像，不创建或启动它们。
if command -v docker >/dev/null 2>&1; then
  docker version
  docker compose version
  docker ps --no-trunc
  docker images --digests
else
  echo "docker: not installed"
fi

section "ROS DDS ENVIRONMENT"
# 查询 ROS 命令、工作区入口和可能影响 DDS 发现的环境变量。
command -v ros2
command -v roscore
find /opt/ros -maxdepth 2 -type f -name setup.bash -print 2>/dev/null
env | grep -E '^(ROS|RMW|CYCLONE|FASTRTPS|AMENT|COLCON|UNITREE)' | sort

section "RUNNING SERVICES"
# 只读取 systemd 状态，重点关注可能占用运动/传感器通道的原厂服务。
systemctl --failed --no-pager
systemctl list-units --type=service --state=running --no-pager | grep -Ei 'unitree|sport|loco|motion|dds|cyclone|ros|camera|lidar|imu|docker|container'

section "RELEVANT PROCESSES"
# 记录实时优先级、父子关系和完整命令行，辅助判断数据/控制通道所有者。
ps -eo user,pid,ppid,rtprio,ni,stat,lstart,args --sort=pid | grep -Ei 'unitree|sport|loco|motion|dds|cyclone|ros|camera|lidar|imu|docker|containerd' | grep -v grep

section "LISTENING SOCKETS"
# 只读列出监听端口及进程，排查端口冲突和远程服务暴露情况。
ss -lntup

section "INSTALLED ROBOTICS PACKAGES"
# 从 dpkg 数据库读取机器人、DDS、容器、GPU 和实时相关包版本。
dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null | grep -Ei 'unitree|ros-|cyclone|fastdds|rmw|docker|nvidia|cuda|realtime'

section "LIKELY UNITREE AND ROS WORKSPACES"
# 限深查找可能的 SDK/ROS 工作区目录；不遍历或修改其文件内容。
for root in /home/unitree /opt /usr/local; do
  if [[ -d "${root}" ]]; then
    find "${root}" -maxdepth 4 -type d \
      \( -iname '*unitree*' -o -iname '*sdk2*' -o -iname '*ros2*' \
         -o -iname '*cyclone*' -o -iname '*loco*' -o -iname '*sport*' \) \
      -print 2>/dev/null
  fi
done

section "INVENTORY COMPLETE"
# 机器可解析的结束标记；是否满足部署条件仍需检查前面各节。
echo "READ_ONLY_PC2_INVENTORY_OK"
