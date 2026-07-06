#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"

echo "[build] root: ${ROOT}"
echo "[build] image: ${IMAGE}"
docker build -t "${IMAGE}" "${ROOT}"

echo "[build] done"
docker images | grep -E 'REPOSITORY|x30_livox|jezetek' || true
