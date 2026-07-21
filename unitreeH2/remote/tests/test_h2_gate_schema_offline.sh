#!/usr/bin/env bash

# Pure offline regression for the Stage 06C/06D/06E gate-file schemas.
# It sources validation helpers only and never calls DDS, an H2 API, or a
# control command.

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
    'timestamp=2026-07-16T00:00:00+08:00' \
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
    'fsm_mode=-1' \
    'observer=NO_UNEXPECTED_MOTION_OBSERVED' \
    'nonzero_velocity_invoked=0' \
    'zero_stop_rpc_ok=1' \
    'parent_stage06c_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb'
} >"$fixture_dir/06d.ok"

{
  write_common 06e
  printf '%s\n' \
    'axis=x-positive' \
    'linear_speed=0.080' \
    'yaw_speed=0.080' \
    'stream_ms=1000' \
    'command_hz=20' \
    'command_period_ms=50' \
    'max_send_gap_ms=100' \
    'expected_rpc_count=20' \
    'rpc_count=20' \
    'max_observed_send_gap_ms=50' \
    'vendor_duration_s=0.300' \
    'watchdog_ms=150' \
    'observed_physical_direction=fixture-only' \
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
  0.080 0.080 250 20 50 100 5 5 0.300 150 100

# Regression: durable tee logging must not make Stage 06D reject a real TTY.
grep -Fx '  exec 3>&1' "$SCRIPT_DIR/../h2_pc2_hal_gate_common.sh" >/dev/null
grep -Fx '[[ -t 0 && -t 3 ]] || h2_die STAGE06D_REQUIRES_INTERACTIVE_TTY 40' \
  "$SCRIPT_DIR/../07_pc2_h2_zero_stop_gate.sh" >/dev/null
grep -Fx '[[ -t 0 && -t 3 ]] || h2_die STAGE06E_REQUIRES_INTERACTIVE_TTY 50' \
  "$SCRIPT_DIR/../08_pc2_h2_single_axis_motion_gate.sh" >/dev/null

expect_rejected() {
  local stage="$1"
  local path="$2"
  if (h2_validate_gate_schema "$stage" "$path") >/dev/null 2>&1; then
    printf 'Malformed Stage %s fixture was accepted: %s\n' "$stage" "$path" >&2
    exit 1
  fi
}

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
  NO_UNEXPECTED_MOTION_OBSERVED; do
  if h2_observation_reports_no_motion "$physical_observation"; then
    printf 'Physical/safety observation was misclassified as no motion: %s\n' \
      "$physical_observation" >&2
    exit 1
  fi
done

cp "$fixture_dir/06c.ok" "$fixture_dir/06c-extra.ok"
printf 'unexpected=1\n' >>"$fixture_dir/06c-extra.ok"
expect_rejected 06c "$fixture_dir/06c-extra.ok"

cp "$fixture_dir/06d.ok" "$fixture_dir/06d-key.ok"
sed -i 's/^zero_stop_rpc_ok=/wrong_key=/' "$fixture_dir/06d-key.ok"
expect_rejected 06d "$fixture_dir/06d-key.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-axis.ok"
sed -i 's/^axis=.*/axis=forward/' "$fixture_dir/06e-axis.ok"
expect_rejected 06e "$fixture_dir/06e-axis.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-speed.ok"
sed -i 's/^linear_speed=.*/linear_speed=fast/' "$fixture_dir/06e-speed.ok"
expect_rejected 06e "$fixture_dir/06e-speed.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-speed-high.ok"
sed -i 's/^linear_speed=.*/linear_speed=0.101/' "$fixture_dir/06e-speed-high.ok"
expect_rejected 06e "$fixture_dir/06e-speed-high.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-stream-high.ok"
sed -i 's/^stream_ms=.*/stream_ms=1050/' "$fixture_dir/06e-stream-high.ok"
expect_rejected 06e "$fixture_dir/06e-stream-high.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-stream-step.ok"
sed -i 's/^stream_ms=.*/stream_ms=975/' "$fixture_dir/06e-stream-step.ok"
expect_rejected 06e "$fixture_dir/06e-stream-step.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-command-hz.ok"
sed -i 's/^command_hz=.*/command_hz=10/' "$fixture_dir/06e-command-hz.ok"
expect_rejected 06e "$fixture_dir/06e-command-hz.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-command-period.ok"
sed -i 's/^command_period_ms=.*/command_period_ms=40/' \
  "$fixture_dir/06e-command-period.ok"
expect_rejected 06e "$fixture_dir/06e-command-period.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-send-gap-limit.ok"
sed -i 's/^max_send_gap_ms=.*/max_send_gap_ms=101/' \
  "$fixture_dir/06e-send-gap-limit.ok"
expect_rejected 06e "$fixture_dir/06e-send-gap-limit.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-expected-rpc.ok"
sed -i 's/^expected_rpc_count=.*/expected_rpc_count=19/' \
  "$fixture_dir/06e-expected-rpc.ok"
expect_rejected 06e "$fixture_dir/06e-expected-rpc.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-rpc-count.ok"
sed -i 's/^rpc_count=.*/rpc_count=19/' "$fixture_dir/06e-rpc-count.ok"
expect_rejected 06e "$fixture_dir/06e-rpc-count.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-observed-gap.ok"
sed -i 's/^max_observed_send_gap_ms=.*/max_observed_send_gap_ms=101/' \
  "$fixture_dir/06e-observed-gap.ok"
expect_rejected 06e "$fixture_dir/06e-observed-gap.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-vendor-duration.ok"
sed -i 's/^vendor_duration_s=.*/vendor_duration_s=0.250/' \
  "$fixture_dir/06e-vendor-duration.ok"
expect_rejected 06e "$fixture_dir/06e-vendor-duration.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-watchdog.ok"
sed -i 's/^watchdog_ms=.*/watchdog_ms=151/' "$fixture_dir/06e-watchdog.ok"
expect_rejected 06e "$fixture_dir/06e-watchdog.ok"

for no_motion_observation in \
  NO_VISIBLE_MOTION \
  NO_PHYSICAL_MOTION_OBSERVED \
  NO_VISIBLE_MOVEMENT_OBSERVED; do
  no_motion_fixture="$fixture_dir/06e-no-motion-${no_motion_observation}.ok"
  cp "$fixture_dir/06e.ok" "$no_motion_fixture"
  sed -i \
    "s/^observed_physical_direction=.*/observed_physical_direction=$no_motion_observation/" \
    "$no_motion_fixture"
  expect_rejected 06e "$no_motion_fixture"
done

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-observer.ok"
sed -i 's/^observer=.*/observer=NOT_CONFIRMED/' "$fixture_dir/06e-observer.ok"
expect_rejected 06e "$fixture_dir/06e-observer.ok"

cp "$fixture_dir/06e.ok" "$fixture_dir/06e-stream-flag.ok"
sed -i 's/^single_axis_stream_rpc_ok=.*/single_axis_stream_rpc_ok=0/' \
  "$fixture_dir/06e-stream-flag.ok"
expect_rejected 06e "$fixture_dir/06e-stream-flag.ok"

motion_gate="$SCRIPT_DIR/../08_pc2_h2_single_axis_motion_gate.sh"
observation_log_line="$(grep -n '^observation_log=' "$motion_gate" | cut -d: -f1)"
no_motion_gate_line="$(grep -n '^if h2_observation_reports_no_motion ' \
  "$motion_gate" | cut -d: -f1)"
safety_prompt_line="$(grep -n "Type BOUNDED_STREAM_OBSERVED_SAFE exactly" \
  "$motion_gate" | cut -d: -f1)"
write_gate_line="$(grep -n '^h2_write_gate 06e ' "$motion_gate" | cut -d: -f1)"
[[ -n "$observation_log_line" && -n "$no_motion_gate_line" &&
   -n "$safety_prompt_line" && -n "$write_gate_line" ]] || {
  printf 'Stage 06E no-motion control-flow markers are missing.\n' >&2
  exit 1
}
(( observation_log_line < no_motion_gate_line &&
   no_motion_gate_line < safety_prompt_line &&
   safety_prompt_line < write_gate_line )) || {
  printf 'Stage 06E no-motion gate must follow logging and precede success certification.\n' >&2
  exit 1
}
grep -F 'H2_STAGE06E_NO_MOTION_OBSERVED axis=' "$motion_gate" >/dev/null
grep -F 'h2_die PHYSICAL_MOTION_NOT_OBSERVED_NO_STAGE06E_GATE 52' \
  "$motion_gate" >/dev/null
grep -F '[[ "$safety_observation" == BOUNDED_STREAM_OBSERVED_SAFE ]] ||' \
  "$motion_gate" >/dev/null
grep -F 'H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK axis=$axis' \
  "$motion_gate" >/dev/null
grep -F "printf -v final_phrase 'RUN_STREAM_%s_L%s_Y%s_S%s_H%s'" \
  "$motion_gate" >/dev/null
grep -F 'single_axis_stream_rpc_ok=1' "$motion_gate" >/dev/null

printf 'H2_GATE_SCHEMA_OFFLINE_OK\n'
