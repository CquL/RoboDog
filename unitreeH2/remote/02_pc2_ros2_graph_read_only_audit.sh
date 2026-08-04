#!/usr/bin/env bash

# Unitree H2 PC2 ROS 2 图谱只读发现脚本。
# 用途：在与原厂进程相同的 DDS 网卡/Domain 上枚举节点、话题、服务和 QoS。
# 边界：ros2 CLI 会创建临时 DDS participant 并发送发现流量，但不会发布
# 机器人命令、调用服务、发送 action goal 或初始化 SDK2 控制客户端。

# 图中个别命令超时/失败时仍继续记录其他端点。
set +e

# 输出稳定分节，便于保存和比对审计日志。
section() {
  printf '\n===== %s =====\n' "$1"
}

# 加载 ROS 2 Humble；若原厂 graph_pid_ws 存在，再加载其自定义消息类型。
source /opt/ros/humble/setup.bash
if [[ -f /home/unitree/graph_pid_ws/install/setup.bash ]]; then
  source /home/unitree/graph_pid_ws/install/setup.bash
fi

# 与交付机器上正在运行的 ROS 2 进程保持一致：CycloneDDS XML 绑定 eth0，
# 适用于任意 DDS Domain；未观察到原厂 ROS_DOMAIN_ID 覆盖，因此本审计固定
# 使用 ROS 2 默认 Domain 0。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0

# 禁止使用持久 ros2 CLI daemon，确保审计结束后不遗留长期 DDS participant。
export ROS2CLI_DISABLE_DAEMON=1

section "TIME AND ROS ENVIRONMENT"
# 记录发现上下文，避免在错误的 RMW、XML 或 Domain 下解释空图。
date --iso-8601=seconds
env | grep -E '^(ROS|RMW|CYCLONE|FASTRTPS|AMENT|COLCON|UNITREE)' | sort

section "ROS2 CLI DAEMON BEFORE"
# 审计开始前记录是否已有 daemon；这里只查询，不主动终止第三方进程。
pgrep -af 'ros2.*daemon' || true

section "UNITREE ROS2 PACKAGES AND INTERFACES"
# 枚举本机可解码的机器人/传感器消息包；包存在不等于实机有对应硬件。
ros2 pkg list | grep -Ei 'unitree|rslidar|hesai|livox|lio_sam' | sort
ros2 interface list | grep -Ei 'unitree|rslidar|hesai|livox' | sort

section "NODES"
# 有界等待 DDS 发现，列出节点（含隐藏节点）。
timeout 20s ros2 node list --no-daemon --spin-time 5 -a

section "TOPICS WITH TYPES"
# 记录普通话题及其类型。
timeout 20s ros2 topic list --no-daemon --spin-time 5 -t

section "TOPICS INCLUDING HIDDEN"
# 单独列出隐藏话题，避免漏掉原厂内部端点。
timeout 20s ros2 topic list --no-daemon --spin-time 5 -t --include-hidden-topics

section "TOPIC GRAPH VERBOSE"
# verbose 输出发布/订阅端点与 QoS，只做发现，不订阅样本。
timeout 30s ros2 topic list --no-daemon --spin-time 5 -v

section "SERVICES WITH TYPES"
# 只枚举服务，不发起任何请求。
timeout 20s ros2 service list --no-daemon --spin-time 5 -t

# Humble 的 `ros2 action list` 没有 --no-daemon 选项，并已观察到它会启动
# 持久 CLI daemon，因此本只读、无残留审计刻意省略 action 发现。

section "ROS2 CLI DAEMON AFTER"
# 结束时再次确认没有由本脚本遗留的 daemon。
pgrep -af 'ros2.*daemon' || true

section "AUDIT COMPLETE"
# 稳定结束标记不等于所有话题都有活跃样本，也不证明控制 RPC 可用。
echo "READ_ONLY_PC2_ROS2_GRAPH_AUDIT_OK"
