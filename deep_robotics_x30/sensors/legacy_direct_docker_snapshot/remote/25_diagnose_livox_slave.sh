#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
NAME="${NAME:-x30_ros2_livox_slave_diag}"
CONFIG_PATH="${CONFIG_PATH:-/config/x30_multi_mid360_ros2_slave.json}"
DURATION="${DURATION:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
PCAP="/home/ysc/x30_livox_slave_diag_${STAMP}.pcap"
TCPDUMP_PID=""

cleanup() {
  if [ -n "${TCPDUMP_PID}" ]; then
    sudo kill "${TCPDUMP_PID}" >/dev/null 2>&1 || true
    wait "${TCPDUMP_PID}" >/dev/null 2>&1 || true
  fi
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[slave-diag] Read-only sensor diagnostic. No robot command is sent."
echo "[slave-diag] Factory ROS1 remains the Livox master."

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

if ! timeout 8 rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; then
  echo "[slave-diag] ERROR: factory /lidar_points has no data." >&2
  exit 1
fi

if ! command -v tcpdump >/dev/null 2>&1; then
  echo "[slave-diag] ERROR: tcpdump is not installed." >&2
  exit 1
fi

IFACE="$(ip route get 192.168.2.202 | awk '{for (i=1; i<=NF; ++i) if ($i=="dev") {print $(i+1); exit}}')"
if [ -z "${IFACE}" ]; then
  echo "[slave-diag] ERROR: cannot find interface for 192.168.2.202." >&2
  exit 1
fi

echo "[slave-diag] Interface: ${IFACE}"
echo "[slave-diag] PCAP: ${PCAP}"
sudo -v

cleanup
sudo timeout "${DURATION}" tcpdump -ni "${IFACE}" -s 128 -w "${PCAP}" \
  'dst host 224.1.1.5 and udp and (dst port 56301 or dst port 56401)' \
  >/tmp/x30_livox_slave_tcpdump.out 2>/tmp/x30_livox_slave_tcpdump.err &
TCPDUMP_PID=$!

docker run -d \
  --name "${NAME}" \
  --network host \
  --ipc=host \
  -v "${ROOT}/config:/config:ro" \
  "${IMAGE}" \
  ros2 launch x30_livox_tools x30_all_sensors_launch.py \
    config_path:="${CONFIG_PATH}" \
    xfer_format:=0 \
    multi_topic:=0 \
    publish_freq:=10.0 \
    lidar_frame_id:=lidar_link \
    cloud_input_topic:=/livox/lidar \
    cloud_output_topic:=/x30/points_merged \
    cloud_output_frame:=lidar_link \
    cloud_window_ms:=100.0 \
    cloud_min_clouds:=1 \
    enable_body_imu:=false >/dev/null

sleep 5

echo
echo "=== Container ==="
docker ps --filter "name=^/${NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'

echo
echo "=== Livox UDP sockets while both processes run ==="
sudo ss -unlp | grep -E '56101|56201|56301|56401|44332|224\.1\.1\.5' || true

echo
echo "=== Kernel IGMP memberships ==="
cat /proc/net/igmp

echo
echo "=== Interface multicast addresses (${IFACE}) ==="
ip maddr show dev "${IFACE}"

echo
echo "=== ROS2 topic endpoints ==="
docker exec "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic info /livox/lidar -v || true
ros2 topic info /x30/points_merged -v || true
'

wait "${TCPDUMP_PID}" || true
sudo chown "$(id -u):$(id -g)" "${PCAP}" 2>/dev/null || true

echo
echo "=== Captured multicast packets ==="
sudo tcpdump -nn -r "${PCAP}" 2>/dev/null | awk '
  /\.56301: UDP/ {point++}
  /\.56401: UDP/ {imu++}
  END {
    printf("point_port_56301=%d\n", point + 0)
    printf("imu_port_56401=%d\n", imu + 0)
  }
'

echo
echo "=== Factory ROS1 still receives ==="
if timeout 8 rostopic echo -n 1 /lidar_points/header >/dev/null 2>&1; then
  echo "ROS1 /lidar_points: PASS"
else
  echo "ROS1 /lidar_points: FAIL"
fi

echo
echo "=== ROS2 receives ==="
if docker exec "${NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
timeout 5 ros2 topic echo --once /livox/lidar --field header >/dev/null
'; then
  echo "ROS2 /livox/lidar: PASS"
else
  echo "ROS2 /livox/lidar: FAIL"
fi

echo
echo "=== Slave logs ==="
docker logs --tail 160 "${NAME}" 2>&1 || true

echo
echo "[slave-diag] Diagnostic complete. Slave container will now be removed."
echo "[slave-diag] PCAP kept at: ${PCAP}"
