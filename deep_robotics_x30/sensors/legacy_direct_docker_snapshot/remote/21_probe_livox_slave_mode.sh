#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
SLAVE_CONFIG="${ROOT}/config/x30_multi_mid360_ros2_slave.json"

echo "[slave-probe] Read-only probe. No ROS node or robot command will be started."

if [ ! -f "${SLAVE_CONFIG}" ]; then
  echo "[slave-probe] ERROR: missing ${SLAVE_CONFIG}" >&2
  exit 1
fi

python3 - "${SLAVE_CONFIG}" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    config = json.load(stream)

if config.get("master_sdk") is not False:
    raise SystemExit("slave config must contain master_sdk=false")

if "lidar_log_enable" in config:
    required = {"lidar_log_cache_size_MB", "lidar_log_path"}
    missing = sorted(required.difference(config))
    if missing:
        raise SystemExit(
            "lidar_log_enable requires: " + ", ".join(missing)
        )

host_net_info = config.get("MID360", {}).get("host_net_info")
if not isinstance(host_net_info, list) or len(host_net_info) != 1:
    raise SystemExit("slave MID360.host_net_info must be a one-item array")

expected_lidars = {
    "192.168.2.202",
    "192.168.2.203",
    "192.168.2.204",
    "192.168.2.205",
}
if set(host_net_info[0].get("lidar_ip", [])) != expected_lidars:
    raise SystemExit("slave host_net_info.lidar_ip must list all four X30 lidars")
PY
if ! grep -Eq '"master_sdk"[[:space:]]*:[[:space:]]*false' "${SLAVE_CONFIG}"; then
  echo "[slave-probe] ERROR: slave config must contain master_sdk=false." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[slave-probe] ERROR: Docker image ${IMAGE} does not exist." >&2
  exit 1
fi

echo "=== Livox SDK in image ==="
docker run --rm --entrypoint bash "${IMAGE}" -lc '
set -euo pipefail
sdk=/usr/local/lib/liblivox_lidar_sdk_shared.so
test -f "${sdk}"
ls -lh "${sdk}"
sha256sum "${sdk}"
if grep -aFq "master_sdk" "${sdk}"; then
  echo "SDK binary contains master_sdk support."
else
  echo "ERROR: SDK binary does not expose master_sdk support." >&2
  exit 2
fi
'

source /opt/ros/noetic/setup.bash
source /home/ysc/jy_cog/drivers/setup.bash 2>/dev/null || true
source /home/ysc/jy_cog/system/devel/setup.bash 2>/dev/null || true

echo
echo "=== Factory ROS1 Livox nodes ==="
rosnode list 2>/dev/null | grep -i livox || true

echo
echo "=== Factory ROS1 point topic ==="
rostopic info /lidar_points 2>/dev/null || true

echo
echo "=== Livox UDP sockets ==="
ss -unlp | grep -E '56101|56201|56301|56401|56501|224\.1\.1\.5' || true

echo
echo "[slave-probe] Probe complete. This did not bind Livox ports."
echo "[slave-probe] Next, only when factory /lidar_points is healthy:"
echo "  bash remote/22_run_ros2_livox_slave.sh"
