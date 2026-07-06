#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_livox_ros2}"
docker rm -f x30_body_imu >/dev/null 2>&1 || true
docker rm -f x30_cloud_merger >/dev/null 2>&1 || true
docker rm -f "${NAME}" >/dev/null 2>&1 || true
echo "[ros2] stopped ${NAME}"
