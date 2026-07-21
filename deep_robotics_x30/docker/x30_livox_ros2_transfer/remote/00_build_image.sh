#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"

echo "[build] root: ${ROOT}"
echo "[build] image: ${IMAGE}"
docker build -t "${IMAGE}" "${ROOT}"

echo "[build] verifying Livox SDK multicast slave support..."
docker run --rm --entrypoint bash "${IMAGE}" -lc '
set -euo pipefail
sdk=/usr/local/lib/liblivox_lidar_sdk_shared.so
test -f "${sdk}"
grep -aFq "master_sdk" "${sdk}"
sha256sum "${sdk}"
echo "[build] Livox SDK master_sdk support verified."
'

echo "[build] verifying offline X30 plane segmentation core..."
docker run --rm --network none --entrypoint bash "${IMAGE}" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
dpkg-query -W libpcl-dev libeigen3-dev
ros2 pkg prefix x30_plane_seg_core
test -f /ws/install/x30_plane_seg_core/lib/libx30_plane_seg_core.so
ctest --test-dir /ws/build/x30_plane_seg_core \
  --output-on-failure \
  --no-tests=error \
  -R "^x30_plane_seg_core_.*_test$"
ldd /ws/install/x30_plane_seg_core/lib/libx30_plane_seg_core.so
echo "[build] x30_plane_seg_core offline contract verified."
'

echo "[build] done"
docker images | grep -E 'REPOSITORY|x30_livox|jezetek' || true
