#!/bin/bash
set -e

# 为默认 launch 命令和 docker exec shell 加载基础 Humble 及本镜像的
# 加载 colcon 工作空间 overlay。
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

# 使用请求的进程替换 shell，保留 Docker 信号处理。
# 此处不会隐式启动传感器或机器人控制命令。
exec "$@"
