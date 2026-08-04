#!/usr/bin/env bash

# Unitree H2 PC2 重点话题契约与活性只读审计。
# 用途：核对 H2/HG 消息定义、SDK2 工件、选定端点/QoS，并在有限时间内
# 被动订阅 IMU、里程计、点云和 TF。输出由调用者保存为证据日志。
# 历史原因：早期 graph 审计中的 `ros2 action list` 曾启动 CLI daemon；
# 本脚本只停止该 CLI daemon，然后坚持无 daemon 查询。
# 安全边界：绝不发布、调用服务、发送 action goal 或调用 SDK 控制 RPC。

# 允许单项失败并继续收集完整契约；每项返回码由 run_check 写入日志。
set +e

# 日志分节标题。
section() {
  printf '\n===== %s =====\n' "$1"
}

# 包装只读命令：保留真实返回码，但返回 0 让审计继续。
# label 是稳定日志键，便于后续脚本逐项判断，而不是只看末尾 OK。
run_check() {
  local label="$1"
  shift
  "$@"
  local rc=$?
  printf 'COMMAND_RC[%s]=%s\n' "$label" "$rc"
  return 0
}

# 加载 Humble 和原厂自定义接口，使 ros2 CLI 能反序列化 Unitree 消息。
source /opt/ros/humble/setup.bash
if [[ -f /home/unitree/graph_pid_ws/install/setup.bash ]]; then
  source /home/unitree/graph_pid_ws/install/setup.bash
fi

# 与原厂进程保持同一 CycloneDDS/eth0/Domain 0 发现上下文。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
export ROS2CLI_DISABLE_DAEMON=1

section "TIME AND ENVIRONMENT"
# 记录审计时间和所有影响 ROS/DDS 行为的环境变量。
date --iso-8601=seconds
env | grep -E '^(ROS|RMW|CYCLONE|AMENT|COLCON|UNITREE)' | sort

section "CLEAN UP AUDIT-CREATED ROS2 CLI DAEMON"
# 只针对 ros2cli.daemon.daemonize；不停止原厂机器人、SLAM 或传感器服务。
pgrep -af 'ros2cli\.daemon\.daemonize' || true
if pgrep -f 'ros2cli\.daemon\.daemonize' >/dev/null 2>&1; then
  run_check "ros2_daemon_stop_before" ros2 daemon stop
fi
pgrep -af 'ros2cli\.daemon\.daemonize' || echo "ROS2_CLI_DAEMON_NOT_RUNNING"

section "ETH0 NEIGHBORS"
# 只读确认 eth0 邻居，不主动 ping 或修改 ARP/NDP。
run_check "ip_neigh_eth0" ip neigh show dev eth0

section "LOCAL H2 MESSAGE SUPPORT"
# 验证本机已安装 request/response 和 HG 状态消息定义；这是解码前置条件，
# 并不触发任何话题订阅或 RPC。
run_check "pkg_unitree_api" ros2 pkg prefix unitree_api
run_check "interface_unitree_api_request" ros2 interface show unitree_api/msg/Request
run_check "interface_unitree_api_response" ros2 interface show unitree_api/msg/Response
run_check "pkg_unitree_hg" ros2 pkg prefix unitree_hg
run_check "interface_unitree_hg_lowstate" ros2 interface show unitree_hg/msg/LowState
run_check "interface_unitree_hg_imustate" ros2 interface show unitree_hg/msg/IMUState
run_check "interface_unitree_hg_bmsstate" ros2 interface show unitree_hg/msg/BmsState

section "H2/HG TYPE AND SDK2 ARTIFACTS"
# 在既有目录中查找 H2/HG IDL 与 LocoClient 头文件，判断本机开发能力。
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

# 对固定候选工件记录符号链接目标、文件格式和哈希，识别重复安装或 ABI 漂移。
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
# 候选列表覆盖控制 API、HG 状态、标准 IMU/里程计、点云/TF、相机以及
# 明确禁止写入的 lowcmd/arm_sdk；这里仅调用 topic info，不发送数据。
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
# 对标准只读输出做 10 秒有界频率观测。timeout 返回 124 是窗口自然结束，
# run_check 会原样记录，而不会把它误当成运动/系统故障。
for topic in /dog_imu_raw /dog_odom /point_in_map /tf; do
  printf '\n--- %s ---\n' "${topic}"
  # timeout returns 124 after the bounded observation window; that is expected.
  run_check "topic_hz:${topic}" timeout --signal=INT 10s ros2 topic hz --window 50 --wall-time "${topic}"
done

section "STANDARD IMU SAMPLE"
# 只接收一帧标准 IMU，验证消息能实际反序列化。
run_check "topic_echo:/dog_imu_raw" timeout 10s ros2 topic echo --no-daemon --spin-time 5 --once /dog_imu_raw

section "STANDARD ODOMETRY SAMPLE"
# 只接收一帧里程计，验证话题存在之外的样本活性。
run_check "topic_echo:/dog_odom" timeout 10s ros2 topic echo --no-daemon --spin-time 5 --once /dog_odom

section "ROS2 CLI DAEMON FINAL"
# 再次只清理由本类审计可能产生的 CLI daemon，保留所有原厂服务。
pgrep -af 'ros2cli\.daemon\.daemonize' || true
if pgrep -f 'ros2cli\.daemon\.daemonize' >/dev/null 2>&1; then
  run_check "ros2_daemon_stop_after" ros2 daemon stop
fi
pgrep -af 'ros2cli\.daemon\.daemonize' || echo "ROS2_CLI_DAEMON_NOT_RUNNING"

section "AUDIT COMPLETE"
# 结束标记只表示审计脚本完成；各 COMMAND_RC 和样本内容才是验收依据。
echo "READ_ONLY_PC2_TOPIC_CONTRACT_AUDIT_OK"
