#!/usr/bin/env bash

# Pure offline regression for Stage 06C/06D/06E gate schemas and the r9
# MotionSwitcher + /dog_odom control flow. Never calls DDS or a robot API.

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=../h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/../h2_pc2_hal_gate_common.sh"

fixture_dir="$(mktemp -d)"
trap 'rm -rf -- "$fixture_dir"' EXIT

write_common() {
  local stage="$1"
  printf '%s\n' \
    "stage=$stage" \
    'timestamp=2026-07-21T00:00:00+08:00' \
    'hostname=unitree-H2-pc2' \
    'boot_id=00000000-0000-0000-0000-000000000000' \
    'release=/home/unitree/p2_unitreeH2/build/offline-fixture' \
    'manifest_sha256=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' \
    "log=/home/unitree/p2_unitreeH2/logs/h2_pc2_stage${stage}_fixture.log"
}

{
  write_common 06c
  printf '%s\n' \
    'fsm_id=601' \
    'fsm_mode=0' \
    'mode_value_observation_only=1' \
    'control_setter_invoked=0' \
    'getter_only_rpc_ok=1'
} >"$fixture_dir/06c.ok"

{
  write_common 06d
  printf '%s\n' \
    'fsm_id=601' \
    'fsm_mode=0' \
    'observer=NO_UNEXPECTED_MOTION_OBSERVED' \
    'nonzero_velocity_invoked=0' \
    'zero_stop_rpc_ok=1' \
    'parent_stage06c_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb'
} >"$fixture_dir/06d.ok"

{
  write_common 06e
  printf '%s\n' \
    'axis=x-positive' \
    'linear_speed=0.100' \
    'yaw_speed=0.080' \
    'stream_ms=1000' \
    'command_hz=20' \
    'command_period_ms=50' \
    'max_send_gap_ms=100' \
    'expected_rpc_count=20' \
    'rpc_count=20' \
    'max_observed_send_gap_ms=51' \
    'vendor_duration_s=0.300' \
    'watchdog_ms=150' \
    'observed_physical_direction=X_POSITIVE_MATCHED_ROBOT_FORWARD' \
    'fsm_id=601' \
    'observer=BOUNDED_STREAM_OBSERVED_SAFE' \
    'state_changing_action_invoked=0' \
    'single_axis_stream_rpc_ok=1' \
    'parent_stage06c_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb' \
    'parent_stage06d_sha256=cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc'
} >"$fixture_dir/06e.ok"

h2_validate_gate_schema 06c "$fixture_dir/06c.ok"
h2_validate_gate_schema 06d "$fixture_dir/06d.ok"
h2_validate_gate_schema 06e "$fixture_dir/06e.ok"
h2_validate_stage06e_stream_profile \
  0.100 0.080 1000 20 50 100 20 20 0.300 150 51

expect_rejected() {
  local stage="$1"
  local path="$2"
  if (h2_validate_gate_schema "$stage" "$path") >/dev/null 2>&1; then
    printf 'Malformed Stage %s fixture was accepted: %s\n' "$stage" "$path" >&2
    exit 1
  fi
}

expect_profile_rejected() {
  if (h2_validate_stage06e_stream_profile "$@") >/dev/null 2>&1; then
    printf 'Unsafe Stage 06E profile was accepted: %s\n' "$*" >&2
    exit 1
  fi
}

# 0.90 m/s must remain outside Stage 06E.
expect_profile_rejected \
  0.900 0.080 1000 20 50 100 20 20 0.300 150 51

for no_motion_observation in \
  NO_MOTION \
  NO_MOTION_OBSERVED \
  NO_VISIBLE_MOTION \
  NO_VISIBLE_MOTION_OBSERVED \
  NO_PHYSICAL_MOTION \
  NO_PHYSICAL_MOTION_OBSERVED \
  NO_VISIBLE_MOVEMENT \
  NO_PHYSICAL_MOVEMENT_OBSERVED; do
  h2_observation_reports_no_motion "$no_motion_observation" || {
    printf 'No-motion observation was not recognized: %s\n' \
      "$no_motion_observation" >&2
    exit 1
  }
done

for physical_observation in \
  X_POSITIVE_MATCHED_ROBOT_FORWARD \
  ROBOT_LEGS_BEGAN_STEPPING \
  BOUNDED_STREAM_OBSERVED_SAFE; do
  if h2_observation_reports_no_motion "$physical_observation"; then
    printf 'Physical observation was misclassified as no motion: %s\n' \
      "$physical_observation" >&2
    exit 1
  fi
done

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-no-motion.ok"
sed -i \
  's/^observed_physical_direction=.*/observed_physical_direction=NO_PHYSICAL_MOTION_OBSERVED/' \
  "$fixture_dir/06e-no-motion.ok"
expect_rejected 06e "$fixture_dir/06e-no-motion.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-speed-high.ok"
sed -i 's/^linear_speed=.*/linear_speed=0.900/' \
  "$fixture_dir/06e-speed-high.ok"
expect_rejected 06e "$fixture_dir/06e-speed-high.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-rpc-count.ok"
sed -i 's/^rpc_count=.*/rpc_count=19/' "$fixture_dir/06e-rpc-count.ok"
expect_rejected 06e "$fixture_dir/06e-rpc-count.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-fsm.ok"
sed -i 's/^fsm_id=.*/fsm_id=1/' "$fixture_dir/06e-fsm.ok"
expect_rejected 06e "$fixture_dir/06e-fsm.ok"

motion_gate="$SCRIPT_DIR/../08_pc2_h2_single_axis_motion_gate.sh"
odom_capture="$SCRIPT_DIR/../h2_dog_odom_capture.py"

[[ -f "$motion_gate" && -f "$odom_capture" ]] || {
  printf 'r9 Stage 06E files are missing.\n' >&2
  exit 1
}

# Parse Python without importing ROS packages or writing __pycache__.
python3 -B -c \
  'import pathlib,sys; p=pathlib.Path(sys.argv[1]); compile(p.read_text(), str(p), "exec")' \
  "$odom_capture"

# Regression markers for the new closed-loop evidence path.
grep -F 'robot_test_unitree_h2_motion_mode' "$motion_gate" >/dev/null
grep -F 'h2_dog_odom_capture.py' "$motion_gate" >/dev/null
grep -F "printf 'baseline\\n'" "$motion_gate" >/dev/null
grep -F "printf 'active\\n'" "$motion_gate" >/dev/null
grep -F "printf 'post\\n'" "$motion_gate" >/dev/null
grep -F 'MOTION_SWITCHER_GATE_OK form=%s name=%s' "$motion_gate" >/dev/null
grep -F 'COMMAND_ACCEPTED_BUT_ODOM_UNCHANGED' "$motion_gate" >/dev/null
grep -F 'ODOM_CHANGED_BUT_PHYSICAL_MOTION_NOT_CONFIRMED' "$motion_gate" >/dev/null
grep -F 'PHYSICAL_MOTION_WITHOUT_ODOM_CONFIRMATION' "$motion_gate" >/dev/null
grep -F 'PHYSICAL_MOTION_OBSERVED or NO_PHYSICAL_MOTION_OBSERVED' \
  "$motion_gate" >/dev/null
grep -F 'odom_response_detected=%s physical_motion_observed=%s' \
  "$motion_gate" >/dev/null
grep -F '0.90 m/s is intentionally rejected' "$motion_gate" >/dev/null

printf 'H2_GATE_SCHEMA_OFFLINE_OK\n'
