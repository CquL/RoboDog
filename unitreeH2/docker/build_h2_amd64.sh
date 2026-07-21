#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"

sdk_dir="${repo_root}/unitreeH2/vendor/unitree_sdk2"
hal_dir="${repo_root}/robot_hardware/robot_hardware"

if [[ ! -f "${sdk_dir}/.source.json" ]]; then
  echo "Missing pinned Unitree SDK2 snapshot: ${sdk_dir}" >&2
  echo "Run unitreeH2/tools/fetch_official_sources.ps1 first." >&2
  exit 2
fi

docker image inspect jezetek:navigation_system_amd64 >/dev/null

docker buildx build \
  --load \
  --pull=false \
  --network=none \
  --build-context "sdk2=${sdk_dir}" \
  --build-context "robot_hardware=${hal_dir}" \
  --file "${script_dir}/Dockerfile" \
  --tag unitree_h2:amd64-offline \
  "${script_dir}"

docker image inspect unitree_h2:amd64-offline \
  --format '{{.Id}} {{.Architecture}} {{index .Config.Labels "io.robodog.unitree_sdk2.commit"}}'
