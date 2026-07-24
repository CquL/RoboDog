#!/usr/bin/env bash

# Simple H2 velocity probe through the single RoboDog HAL test entry point.
# No direct-vendor backend is retained: all motion must pass RobotFactory and
# RobotHardwareInterface.

set -Eeuo pipefail

if [[ $# -ne 4 ]]; then
  printf 'Usage: %s <vx> <vy> <omega> <duration_ms>\n' "$0" >&2
  printf 'Example: %s 0.50 0 0 1000\n' "$0" >&2
  exit 64
fi

vx="$1"
vy="$2"
omega="$3"
duration_ms="$4"

script_dir="$(cd -- "$(dirname -- "$0")" && pwd -P)"
release_dir="$(cd -- "$script_dir/.." && pwd -P)"
binary="$release_dir/bin/robot_test_unitree_h2"
config="$release_dir/config/unitree_h2_live.yaml"
export LD_LIBRARY_PATH="$release_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

stamp="$(date +%Y%m%d_%H%M%S)_$$"
log_dir="${H2_LOG_DIR:-/home/unitree/p2_unitreeH2/logs}"
mkdir -p "$log_dir"
log="$log_dir/h2_pc2_unified_velocity_${stamp}.log"

printf 'H2_UNIFIED_PROBE vx=%s vy=%s omega=%s duration_ms=%s log=%s\n' \
  "$vx" "$vy" "$omega" "$duration_ms" "$log"

set +e
"$binary" --config "$config" --velocity \
  --vx "$vx" --vy "$vy" --omega "$omega" \
  --duration-ms "$duration_ms" --execute 2>&1 | tee "$log"
status=("${PIPESTATUS[@]}")
set -e
[[ "${status[1]}" -eq 0 ]] || exit 70
printf 'H2_UNIFIED_PROBE_RESULT rc=%s log=%s\n' "${status[0]}" "$log"
exit "${status[0]}"
