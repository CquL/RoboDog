#!/usr/bin/env bash

# Unitree H2 PC2 ROS 2 graph discovery only.
# This creates a temporary DDS participant and therefore sends discovery
# traffic, but it does not publish robot commands or call robot services.

set +e

section() {
  printf '\n===== %s =====\n' "$1"
}

source /opt/ros/humble/setup.bash
if [[ -f /home/unitree/graph_pid_ws/install/setup.bash ]]; then
  source /home/unitree/graph_pid_ws/install/setup.bash
fi

# Match the currently running delivered ROS 2 processes. The referenced XML
# binds discovery to eth0 and applies to any DDS domain. The delivered processes
# expose no ROS_DOMAIN_ID override, so this audit pins the ROS 2 default domain 0.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0

# Do not start or use the persistent ros2 CLI daemon for this audit.
export ROS2CLI_DISABLE_DAEMON=1

section "TIME AND ROS ENVIRONMENT"
date --iso-8601=seconds
env | grep -E '^(ROS|RMW|CYCLONE|FASTRTPS|AMENT|COLCON|UNITREE)' | sort

section "ROS2 CLI DAEMON BEFORE"
pgrep -af 'ros2.*daemon' || true

section "UNITREE ROS2 PACKAGES AND INTERFACES"
ros2 pkg list | grep -Ei 'unitree|rslidar|hesai|livox|lio_sam' | sort
ros2 interface list | grep -Ei 'unitree|rslidar|hesai|livox' | sort

section "NODES"
timeout 20s ros2 node list --no-daemon --spin-time 5 -a

section "TOPICS WITH TYPES"
timeout 20s ros2 topic list --no-daemon --spin-time 5 -t

section "TOPICS INCLUDING HIDDEN"
timeout 20s ros2 topic list --no-daemon --spin-time 5 -t --include-hidden-topics

section "TOPIC GRAPH VERBOSE"
timeout 30s ros2 topic list --no-daemon --spin-time 5 -v

section "SERVICES WITH TYPES"
timeout 20s ros2 service list --no-daemon --spin-time 5 -t

# Humble's `ros2 action list` has no --no-daemon option and was observed to
# start a persistent ros2 CLI daemon, so action discovery is deliberately
# omitted from this no-daemon audit.

section "ROS2 CLI DAEMON AFTER"
pgrep -af 'ros2.*daemon' || true

section "AUDIT COMPLETE"
echo "READ_ONLY_PC2_ROS2_GRAPH_AUDIT_OK"
