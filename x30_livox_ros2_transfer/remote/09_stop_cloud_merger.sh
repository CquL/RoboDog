#!/bin/bash
set -euo pipefail

NAME="${NAME:-x30_cloud_merger}"

docker rm -f "${NAME}" >/dev/null 2>&1 || true
echo "[merge] stopped ${NAME}"
