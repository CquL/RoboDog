#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_ros2_livox_slave}"

docker rm -f "${NAME}" >/dev/null 2>&1 || true

echo "[slave-stop] stopped ${NAME}"
echo "[slave-stop] factory ROS1 Livox was not changed by the slave test"
