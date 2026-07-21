#!/usr/bin/env bash

# Unitree H2 PC2 selected topic contract and liveness audit.
# The prior graph audit accidentally caused Humble's `ros2 action list` to
# start a ros2 CLI daemon. This script first stops only that CLI daemon, then
# performs no-daemon graph queries and read-only subscriptions. It never
# publishes, calls a service, sends an action goal or invokes an SDK control RPC.

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

source /opt/ros/humble/setup.bash
if [[ -f /home/unitree/graph_pid_ws/install/setup.bash ]]; then
  source /home/unitree/graph_pid_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
export ROS2CLI_DISABLE_DAEMON=1

section "TIME AND ENVIRONMENT"
date --iso-8601=seconds
env | grep -E '^(ROS|RMW|CYCLONE|AMENT|COLCON|UNITREE)' | sort

section "CLEAN UP AUDIT-CREATED ROS2 CLI DAEMON"
pgrep -af 'ros2cli\.daemon\.daemonize' || true
if pgrep -f 'ros2cli\.daemon\.daemonize' >/dev/null 2>&1; then
  run_check "ros2_daemon_stop_before" ros2 daemon stop
fi
pgrep -af 'ros2cli\.daemon\.daemonize' || echo "ROS2_CLI_DAEMON_NOT_RUNNING"

section "ETH0 NEIGHBORS"
run_check "ip_neigh_eth0" ip neigh show dev eth0

section "LOCAL H2 MESSAGE SUPPORT"
run_check "pkg_unitree_api" ros2 pkg prefix unitree_api
run_check "interface_unitree_api_request" ros2 interface show unitree_api/msg/Request
run_check "interface_unitree_api_response" ros2 interface show unitree_api/msg/Response
run_check "pkg_unitree_hg" ros2 pkg prefix unitree_hg
run_check "interface_unitree_hg_lowstate" ros2 interface show unitree_hg/msg/LowState
run_check "interface_unitree_hg_imustate" ros2 interface show unitree_hg/msg/IMUState
run_check "interface_unitree_hg_bmsstate" ros2 interface show unitree_hg/msg/BmsState

section "H2/HG TYPE AND SDK2 ARTIFACTS"
search_roots=()
for root in /home/unitree /opt /unitree /usr/local; do
  if [[ -d "$root" ]]; then
    search_roots+=("$root")
  fi
done
if (( ${#search_roots[@]} > 0 )); then
  run_check "find_h2_hg_artifacts" timeout 45s find "${search_roots[@]}" -type f \
    \( -name 'LowState.msg' -o -name 'LowState_.hpp' \
       -o -name 'IMUState.msg' -o -name 'IMUState_.hpp' \
       -o -name 'BmsState.msg' -o -name 'BmsState_.hpp' \
       -o -name 'h2_loco_api.hpp' -o -name 'h2_loco_client.hpp' \) \
    -print
fi

sdk_artifacts=(
  /opt/unitree_robotics/include/unitree/robot/h2/loco/h2_loco_api.hpp
  /opt/unitree_robotics/include/unitree/robot/h2/loco/h2_loco_client.hpp
  /opt/unitree_robotics/lib/libunitree_sdk2.a
  /usr/local/include/unitree/robot/h2/loco/h2_loco_api.hpp
  /usr/local/include/unitree/robot/h2/loco/h2_loco_client.hpp
  /usr/local/lib/libunitree_sdk2.a
  /unitree/opt/lib/libddsc.so.0
  /unitree/opt/lib/libddscxx.so.0
  /opt/unitree_robotics/lib/libddsc.so.0
  /opt/unitree_robotics/lib/libddscxx.so.0
)
for artifact in "${sdk_artifacts[@]}"; do
  if [[ -e "$artifact" ]]; then
    printf '\n--- %s ---\n' "$artifact"
    ls -l "$artifact"
    resolved="$(readlink -f "$artifact")"
    printf 'resolved=%s\n' "$resolved"
    file "$resolved"
    sha256sum "$resolved"
  fi
done

section "SELECTED TOPIC ENDPOINTS AND QOS"
topics=(
  /api/sport/request
  /api/sport/response
  /lf/lowstate
  /lowstate_raw
  /lf/secondary_imu
  /secondary_imu
  /lf/bmsstate
  /lf/mainboardstate
  /lf/emergency_stop
  /sportmodestate
  /odommodestate
  /dog_imu_raw
  /dog_odom
  /point_in_map
  /tf
  /frontvideostream
  /lowcmd
  /arm_sdk
)
for topic in "${topics[@]}"; do
  printf '\n--- %s ---\n' "${topic}"
  run_check "topic_info:${topic}" timeout 15s ros2 topic info --no-daemon --spin-time 5 --verbose "${topic}"
done

section "STANDARD TOPIC RECEIVE RATES"
for topic in /dog_imu_raw /dog_odom /point_in_map /tf; do
  printf '\n--- %s ---\n' "${topic}"
  # timeout returns 124 after the bounded observation window; that is expected.
  run_check "topic_hz:${topic}" timeout --signal=INT 10s ros2 topic hz --window 50 --wall-time "${topic}"
done

section "STANDARD IMU SAMPLE"
run_check "topic_echo:/dog_imu_raw" timeout 10s ros2 topic echo --no-daemon --spin-time 5 --once /dog_imu_raw

section "STANDARD ODOMETRY SAMPLE"
run_check "topic_echo:/dog_odom" timeout 10s ros2 topic echo --no-daemon --spin-time 5 --once /dog_odom

section "ROS2 CLI DAEMON FINAL"
pgrep -af 'ros2cli\.daemon\.daemonize' || true
if pgrep -f 'ros2cli\.daemon\.daemonize' >/dev/null 2>&1; then
  run_check "ros2_daemon_stop_after" ros2 daemon stop
fi
pgrep -af 'ros2cli\.daemon\.daemonize' || echo "ROS2_CLI_DAEMON_NOT_RUNNING"

section "AUDIT COMPLETE"
echo "READ_ONLY_PC2_TOPIC_CONTRACT_AUDIT_OK"
