#!/usr/bin/env bash
set -Eeuo pipefail

image="${H2_IMAGE:-unitree_h2:amd64-runtime-candidate}"
container="${H2_IMU_CONTAINER:-unitree-h2-imu-bridge-audit}"
dds_interface="${H2_DDS_INTERFACE:-eth0}"
dds_domain="${H2_DDS_DOMAIN:-0}"
ros_domain="${H2_ROS_DOMAIN_ID:-20}"
ros_log_dir="${H2_ROS_LOG_DIR:-/tmp/ros_logs}"
log_root="/home/unitree/p2_unitreeH2/logs"
stamp="$(date +%Y%m%d_%H%M%S)"
log="${log_root}/h2_docker_imu_bridge_${stamp}.log"

mkdir -p "$log_root"

cleanup() {
  docker stop --time 3 "$container" >/dev/null 2>&1 || true
  docker rm -f "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

exec > >(tee "$log") 2>&1

echo "H2_DOCKER_IMU_AUDIT_MODE=READ_ONLY"
echo "IMAGE=$image"
echo "DDS_INTERFACE=$dds_interface"
echo "DDS_DOMAIN=$dds_domain"
echo "ROS_DOMAIN_ID=$ros_domain"
echo "ROS_LOG_DIR=$ros_log_dir"
echo "LOG=$log"

docker version >/dev/null
docker image inspect "$image" >/dev/null

scope="$(
  docker image inspect "$image" \
    --format '{{index .Config.Labels "io.robodog.h2.runtime.scope"}}'
)"
test "$scope" = "hal-native-hg-state-ros2-imu-candidate"

if docker container inspect "$container" >/dev/null 2>&1; then
  echo "Container already exists: $container" >&2
  exit 20
fi

docker run -d \
  --name "$container" \
  --network host \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  -e "H2_DDS_INTERFACE=$dds_interface" \
  -e "H2_DDS_DOMAIN=$dds_domain" \
  -e "ROS_DOMAIN_ID=$ros_domain" \
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  -e "ROS_LOG_DIR=$ros_log_dir" \
  "$image" \
  /opt/robodog/bin/unitree_h2_sensor_bridge \
    --ros-args \
    --params-file \
    /opt/robodog/share/unitree_h2_sensor_bridge/config/unitree_h2_imu_bridge.yaml \
    -p "dds_interface:=$dds_interface" \
    -p "dds_domain:=$dds_domain"

sleep 4
docker logs "$container"
docker logs "$container" 2>&1 | grep -F "H2_IMU_BRIDGE_READY"

docker exec "$container" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS2CLI_DISABLE_DAEMON=1
  ros2 topic list -t
  timeout 15s ros2 topic echo \
    /h2/imu/pelvis sensor_msgs/msg/Imu --once --no-arr
  timeout 15s ros2 topic echo \
    /h2/imu/torso sensor_msgs/msg/Imu --once --no-arr
'

echo "H2_DOCKER_IMU_BRIDGE_LIVE_SAMPLES_OK"
