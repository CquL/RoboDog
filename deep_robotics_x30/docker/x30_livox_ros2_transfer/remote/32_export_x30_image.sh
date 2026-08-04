#!/bin/bash
set -euo pipefail

# 导出已构建的 amd64 生产镜像，用于迁移到另一台 106。
# 不执行构建、运行或机器人通信。
IMAGE="${IMAGE:-x30_livox_ros2:jezetek}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/x30_image_export}"
readonly ARCHIVE_NAME="x30_livox_ros2_jezetek_amd64.tar"
readonly EXPECTED_ARCH="amd64"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[image-export] ERROR: image ${IMAGE} is not present." >&2
  exit 1
fi

image_arch="$(
  docker image inspect "${IMAGE}" --format '{{.Architecture}}'
)"
# 生成大文件前拒绝意外的 arm64 架构或跨主机混用。
if [[ "${image_arch}" != "${EXPECTED_ARCH}" ]]; then
  echo "[image-export] ERROR: expected ${EXPECTED_ARCH}, got ${image_arch}." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
archive_path="${OUTPUT_DIR}/${ARCHIVE_NAME}"
temporary_path="${archive_path}.tmp"
hash_path="${archive_path}.sha256"

cleanup() {
  # docker save 中断时，不允许以最终文件名留下不完整归档。
  rm -f -- "${temporary_path}"
}
trap cleanup EXIT

echo "[image-export] image:  ${IMAGE}"
echo "[image-export] output: ${archive_path}"
df -h "${OUTPUT_DIR}" /var/lib/docker 2>/dev/null || true

docker save -o "${temporary_path}" "${IMAGE}"
# docker save 成功后才重命名，并生成可移植的 SHA256 校验文件。
mv -f -- "${temporary_path}" "${archive_path}"

(
  cd "${OUTPUT_DIR}"
  sha256sum "${ARCHIVE_NAME}" > "${ARCHIVE_NAME}.sha256"
)

docker image inspect "${IMAGE}" \
  --format 'ID={{.Id}} ARCH={{.Architecture}} TAGS={{json .RepoTags}}'
ls -lh "${archive_path}" "${hash_path}"
cat "${hash_path}"

echo "[image-export] Copy both files into deep_robotics_x30/artifacts."
