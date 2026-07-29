#!/usr/bin/env bash
set -eo pipefail
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

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

/opt/robodog/bin/unitree_h2_factory_contract_test
/opt/robodog/bin/unitree_h2_direct_api_contract_test
/opt/robodog/bin/unitree_h2_live_motion_plan_test
/opt/robodog/bin/unitree_h2_imu_mapping_contract_test

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs
export ROS2CLI_DISABLE_DAEMON=1
mkdir -p "$ROS_LOG_DIR"

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
