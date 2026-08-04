#!/usr/bin/env bash
# 用途：作为 runtime 镜像的最小入口，只准备镜像内 ROS 2 Humble 环境。
# 输入：docker/compose 传入的任意命令及参数。
# 输出：用 exec 启动目标进程，使其直接接收信号并保持正确退出码。
# 安全边界：不 source Jezetek 或宿主机工作区，不启动传感器桥或运动控制，
#           因此“容器已启动”本身不会向 H2 发送控制命令。
set -e

# ROS 2 环境限定在镜像内；SDK2/HAL 自身直接使用 CycloneDDS，并不依赖此 source。
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# 替换 shell 而不是再派生子进程，便于 Docker 正确转发 SIGTERM/SIGINT。
exec "$@"
