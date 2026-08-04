#!/usr/bin/env bash
# 用途：在 Linux/WSL 中用本地固定源码离线构建 H2 HAL-only amd64 镜像。
# 输入：已抓取的 unitree_sdk2、robot_hardware 源码和本地 Jezetek 基础镜像。
# 输出：unitree_h2:amd64-offline，并打印镜像 ID、架构与 SDK2 提交标签。
# 安全边界：--network=none、--pull=false 禁止构建时联网；本脚本只构建镜像，
#           不创建容器、不连接 H2，也不执行任何实机控制。
set -euo pipefail

# 从脚本位置推导项目根目录，避免依赖调用者当前工作目录。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"

sdk_dir="${repo_root}/unitreeH2/vendor/unitree_sdk2"
hal_dir="${repo_root}/robot_hardware/robot_hardware"

# .source.json 是固定官方源码快照的最低来源门禁。
if [[ ! -f "${sdk_dir}/.source.json" ]]; then
  echo "Missing pinned Unitree SDK2 snapshot: ${sdk_dir}" >&2
  echo "Run unitreeH2/tools/fetch_official_sources.ps1 first." >&2
  exit 2
fi

# 构建前确认基础镜像已在本机；不会自动拉取。
docker image inspect jezetek:navigation_system_amd64 >/dev/null

# 命名上下文将 SDK2/HAL 源码送入 Dockerfile，最终结果加载回本地 daemon。
docker buildx build \
  --load \
  --pull=false \
  --network=none \
  --build-context "sdk2=${sdk_dir}" \
  --build-context "robot_hardware=${hal_dir}" \
  --file "${script_dir}/Dockerfile" \
  --tag unitree_h2:amd64-offline \
  "${script_dir}"

# 打印构建结果，供后续离线验收和交付记录核对。
docker image inspect unitree_h2:amd64-offline \
  --format '{{.Id}} {{.Architecture}} {{index .Config.Labels "io.robodog.unitree_sdk2.commit"}}'
