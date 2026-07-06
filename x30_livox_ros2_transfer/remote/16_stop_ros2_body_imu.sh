#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_body_imu}"

docker rm -f "${NAME}" >/dev/null 2>&1 || true
echo "[imu] stopped ${NAME}"
