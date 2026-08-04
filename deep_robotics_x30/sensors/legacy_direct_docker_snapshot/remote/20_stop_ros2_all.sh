#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_ros2_sensors}"

docker rm -f "${NAME}" >/dev/null 2>&1 || true

echo "[all] stopped ${NAME}"
