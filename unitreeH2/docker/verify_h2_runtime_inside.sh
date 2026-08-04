#!/usr/bin/env bash
# 用途：在 runtime-candidate 容器内部完成二进制、依赖、ROS 2 端点和安全门禁验收。
# 输入：镜像内已安装的 SDK2/HAL/状态探针/IMU 桥，以及 /tmp 可写 tmpfs。
# 输出：各契约测试结果、IMU 话题端点检查和 H2_RUNTIME_IMAGE_OFFLINE_OK。
# 安全边界：宿主验证脚本以 --network none 启动本脚本；这里仅使用 lo 启动桥，
#           非零运动测试均验证“被拒绝”，不会与实机 DDS 网络通信。
set -eo pipefail

# 只加载镜像自带 Humble；set -u 在 ROS setup 完成后开启，兼容其可选变量。
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# 清点正式保留的程序/配置，并明确拒绝旧的多入口运动测试二进制。
command -v ros2 >/dev/null
test -x /opt/robodog/bin/robot_test_unitree_h2
test -x /opt/robodog/bin/h2_hg_state_read_only_probe
test -x /opt/robodog/bin/unitree_h2_sensor_bridge
test -x /opt/robodog/bin/unitree_h2_imu_mapping_contract_test
test -x /opt/robodog/bin/unitree_h2_factory_contract_test
test -x /opt/robodog/bin/unitree_h2_direct_api_contract_test
test -x /opt/robodog/bin/unitree_h2_live_motion_plan_test
test -f /opt/robodog/config/unitree_h2_container_safe.yaml
test -f /opt/robodog/config/unitree_h2_container_motion.yaml
test ! -e /opt/robodog/bin/robot_test_unitree_h2_live_motion
test ! -e /opt/robodog/bin/robot_test_unitree_h2_velocity_cli
test ! -e /opt/robodog/bin/robot_test_unitree_h2_vendor_velocity_cli

# 每个核心 ELF 必须没有动态库缺失，否则镜像不得交付。
for elf in \
  /opt/robodog/lib/librobot_hardware.so \
  /opt/robodog/bin/robot_test_unitree_h2 \
  /opt/robodog/bin/h2_hg_state_read_only_probe \
  /opt/robodog/bin/unitree_h2_sensor_bridge \
  /opt/robodog/bin/unitree_h2_imu_mapping_contract_test; do
  if ldd "$elf" | grep -q 'not found'; then
    echo "Missing dependency: $elf" >&2
    exit 21
  fi
done

# 契约测试全部使用离线/fake seam，检查工厂分配、HAL 映射、运动计划与 IMU 字段。
/opt/robodog/bin/unitree_h2_factory_contract_test
/opt/robodog/bin/unitree_h2_direct_api_contract_test
/opt/robodog/bin/unitree_h2_live_motion_plan_test
/opt/robodog/bin/unitree_h2_imu_mapping_contract_test

# 在回环网卡启动 IMU bridge，只验证 ROS 2 节点能够创建两个目标话题。
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs
export ROS2CLI_DISABLE_DAEMON=1
mkdir -p "$ROS_LOG_DIR"

# 后台启动后记录 PID，检查完成主动终止，避免容器验收残留进程。
/opt/robodog/bin/unitree_h2_sensor_bridge \
  --ros-args \
  -p dds_interface:=lo \
  -p dds_domain:=0 \
  -p publish_rate_hz:=20.0 \
  >/tmp/imu_bridge.out 2>&1 &
bridge_pid=$!
sleep 2
ros2 topic list -t >/tmp/imu_topics.out
kill -TERM "$bridge_pid"
wait "$bridge_pid" || true
grep -F 'H2_IMU_BRIDGE_READY' /tmp/imu_bridge.out
grep -F '/h2/imu/pelvis [sensor_msgs/msg/Imu]' /tmp/imu_topics.out
grep -F '/h2/imu/torso [sensor_msgs/msg/Imu]' /tmp/imu_topics.out
echo H2_IMU_BRIDGE_OFFLINE_ENDPOINTS_OK

# 下面故意执行错误/越权参数并捕获返回码：未带 --execute、超限速度、
# 超限角速度和 safe 配置非零速度都必须失败。
set +e
/opt/robodog/bin/h2_hg_state_read_only_probe --seconds 0 >/tmp/state.out 2>&1
state_rc=$?
/opt/robodog/bin/robot_test_unitree_h2 \
  --config /opt/robodog/config/unitree_h2_container_motion.yaml \
  --velocity --vx 1.00 --vy 0 --omega 0.70 --duration-ms 1000 \
  >/tmp/missing_execute.out 2>&1
missing_execute_rc=$?
/opt/robodog/bin/robot_test_unitree_h2 \
  --config /opt/robodog/config/unitree_h2_container_motion.yaml \
  --velocity --vx 1.01 --vy 0 --omega 0 --duration-ms 1000 --execute \
  >/tmp/over_limit.out 2>&1
over_limit_rc=$?
/opt/robodog/bin/robot_test_unitree_h2 \
  --config /opt/robodog/config/unitree_h2_container_motion.yaml \
  --velocity --vx 0 --vy 0 --omega 0.71 --duration-ms 1000 --execute \
  >/tmp/over_omega_limit.out 2>&1
over_omega_limit_rc=$?
/opt/robodog/bin/robot_test_unitree_h2 \
  --config /opt/robodog/config/unitree_h2_container_safe.yaml \
  --velocity --vx 0.20 --vy 0 --omega 0 --duration-ms 1000 --execute \
  >/tmp/safe_profile.out 2>&1
safe_profile_rc=$?
set -e

# 精确核对返回码和诊断标记，防止“程序运行了但门禁失效”被误判为通过。
test "$state_rc" -eq 2
grep -q 'argument outside the accepted safety bounds' /tmp/state.out
test "$missing_execute_rc" -eq 64
test "$over_limit_rc" -eq 65
grep -q 'H2_TEST_CONFIG_WOULD_CLAMP' /tmp/over_limit.out
test "$over_omega_limit_rc" -eq 65
grep -q 'H2_TEST_CONFIG_WOULD_CLAMP' /tmp/over_omega_limit.out
test "$safe_profile_rc" -eq 65
grep -q 'H2_TEST_CONFIG_REJECTED' /tmp/safe_profile.out

echo H2_RUNTIME_IMAGE_OFFLINE_OK
