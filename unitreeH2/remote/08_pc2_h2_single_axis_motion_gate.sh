#!/usr/bin/env bash

# Stage 06E r9: one protected, uncalibrated, bounded single-axis velocity
# stream with MotionSwitcher and /dog_odom evidence.
# Never loops over axes and never calls Start/StandUp/Damp/Squat/Sit.

set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
  printf 'Usage: %s <x-positive|x-negative|y-positive|y-negative|yaw-positive|yaw-negative> [--linear-speed 0.10] [--yaw-speed 0.08] [--stream-ms 1000]\nLimits: linear 0.01..0.10 m/s, yaw 0.01..0.15 rad/s, stream 250..1000 ms in 50 ms steps; fixed command rate 20 Hz. 0.90 m/s is intentionally rejected.\n' \
    "$0" >&2
  exit 64
fi

axis="$1"
shift
case "$axis" in
  x-positive|x-negative|y-positive|y-negative|yaw-positive|yaw-negative) ;;
  *)
    printf 'Rejected uncalibrated or combined axis: %s\n' "$axis" >&2
    exit 64
    ;;
esac

motion_args=(--axis "$axis")
linear_speed_seen=0
yaw_speed_seen=0
stream_ms_seen=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --linear-speed)
      [[ "$linear_speed_seen" -eq 0 ]] || {
        printf 'Duplicate option: %s\n' "$1" >&2
        exit 64
      }
      [[ $# -ge 2 ]] || {
        printf 'Missing value for %s\n' "$1" >&2
        exit 64
      }
      motion_args+=("$1" "$2")
      linear_speed_seen=1
      shift 2
      ;;
    --yaw-speed)
      [[ "$yaw_speed_seen" -eq 0 ]] || {
        printf 'Duplicate option: %s\n' "$1" >&2
        exit 64
      }
      [[ $# -ge 2 ]] || {
        printf 'Missing value for %s\n' "$1" >&2
        exit 64
      }
      motion_args+=("$1" "$2")
      yaw_speed_seen=1
      shift 2
      ;;
    --stream-ms)
      [[ "$stream_ms_seen" -eq 0 ]] || {
        printf 'Duplicate option: %s\n' "$1" >&2
        exit 64
      }
      [[ $# -ge 2 ]] || {
        printf 'Missing value for %s\n' "$1" >&2
        exit 64
      }
      motion_args+=("$1" "$2")
      stream_ms_seen=1
      shift 2
      ;;
    *)
      printf 'Rejected Stage 06E option: %s\n' "$1" >&2
      exit 64
      ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/h2_pc2_hal_gate_common.sh"

authorization_file=""
motion_capture=""
odom_pid=""
odom_phase_file=""
odom_summary=""

cleanup_stage06e() {
  if [[ -n "$odom_pid" ]] && kill -0 "$odom_pid" 2>/dev/null; then
    kill -INT "$odom_pid" 2>/dev/null || true
    wait "$odom_pid" 2>/dev/null || true
  fi
  if [[ -n "$authorization_file" ]]; then
    rm -f -- "$authorization_file"
  fi
  if [[ -n "$motion_capture" ]]; then
    rm -f -- "$motion_capture"
  fi
  if [[ -n "$odom_phase_file" ]]; then
    rm -f -- "$odom_phase_file"
  fi
}
trap cleanup_stage06e EXIT

h2_prepare_gate 06e

command -v python3 >/dev/null 2>&1 || h2_die MISSING_TOOL=python3 11
[[ -x "$H2_BIN_DIR/robot_test_unitree_h2_motion_mode" ]] ||
  h2_die MISSING_EXECUTABLE=robot_test_unitree_h2_motion_mode 16
[[ -r "$SCRIPT_DIR/h2_dog_odom_capture.py" ]] ||
  h2_die H2_DOG_ODOM_CAPTURE_SCRIPT_MISSING 16

h2_require_gate 06c \
  "fsm_id=601" \
  "mode_value_observation_only=1" \
  "control_setter_invoked=0" \
  "getter_only_rpc_ok=1"
stage06c_gate_sha256="$(
  sha256sum "$H2_STATE_DIR/stage06c.ok" | awk '{print $1}'
)"

h2_require_gate 06d \
  "fsm_id=601" \
  "observer=NO_UNEXPECTED_MOTION_OBSERVED" \
  "nonzero_velocity_invoked=0" \
  "zero_stop_rpc_ok=1" \
  "parent_stage06c_sha256=$stage06c_gate_sha256"
stage06d_gate_sha256="$(
  sha256sum "$H2_STATE_DIR/stage06d.ok" | awk '{print $1}'
)"

h2_run_offline_contracts

current_06e="$H2_STATE_DIR/stage06e.ok"
current_axis_history=0
if [[ -e "$current_06e" ]]; then
  h2_require_gate 06e \
    "fsm_id=601" \
    "observer=BOUNDED_STREAM_OBSERVED_SAFE" \
    "state_changing_action_invoked=0" \
    "single_axis_stream_rpc_ok=1" \
    "parent_stage06c_sha256=$stage06c_gate_sha256" \
    "parent_stage06d_sha256=$stage06d_gate_sha256"
  current_axis_history=1
fi
if [[ "$current_axis_history" -eq 0 && "$axis" != x-positive ]]; then
  h2_die FIRST_AXIS_FOR_THIS_BUNDLE_MUST_BE_X_POSITIVE 50
fi

h2_section "PRINT-ONLY PLAN"
plan_output="$(
  h2_isolated "$H2_BIN_DIR/robot_test_unitree_h2_live_motion" \
    --print-plan "${motion_args[@]}" 2>&1
)"
printf '%s\n' "$plan_output"
grep -F H2_LIVE_PRINT_PLAN_ONLY_NO_DDS <<<"$plan_output" >/dev/null ||
  h2_die PRINT_PLAN_GATE_FAILED 50
plan_line="$(grep -F 'H2_LIVE_PLAN axis=' <<<"$plan_output" | tail -n 1)"
[[ -n "$plan_line" ]] || h2_die PRINT_PLAN_LINE_MISSING 50

extract_plan_field() {
  local key="$1"
  if [[ "$plan_line" =~ (^|[[:space:]])${key}=([^[:space:]]+) ]]; then
    printf '%s\n' "${BASH_REMATCH[2]}"
    return 0
  fi
  h2_die "PRINT_PLAN_FIELD_MISSING=$key" 50
}

plan_axis="$(extract_plan_field axis)"
plan_linear_speed="$(extract_plan_field linear_speed)"
plan_yaw_speed="$(extract_plan_field yaw_speed)"
plan_stream_ms="$(extract_plan_field stream_ms)"
plan_command_hz="$(extract_plan_field command_hz)"
plan_command_period_ms="$(extract_plan_field command_period_ms)"
plan_max_send_gap_ms="$(extract_plan_field max_send_gap_ms)"
plan_expected_rpc_count="$(extract_plan_field expected_rpc_count)"
plan_vendor_duration_s="$(extract_plan_field vendor_duration_s)"
plan_watchdog_ms="$(extract_plan_field watchdog_ms)"

[[ "$plan_axis" == "$axis" ]] || h2_die PRINT_PLAN_AXIS_MISMATCH 50
h2_validate_stage06e_stream_profile \
  "$plan_linear_speed" "$plan_yaw_speed" "$plan_stream_ms" \
  "$plan_command_hz" "$plan_command_period_ms" "$plan_max_send_gap_ms" \
  "$plan_expected_rpc_count" "$plan_expected_rpc_count" \
  "$plan_vendor_duration_s" "$plan_watchdog_ms" 0

# h2_prepare_gate redirects stdout through tee; FD 3 remains the original TTY.
[[ -t 0 && -t 3 ]] || h2_die STAGE06E_REQUIRES_INTERACTIVE_TTY 50

h2_section "PHYSICAL MOTION CHECKLIST"
printf '%s\n' \
  'Required: official fall-arrest/protective stand installed.' \
  'Required: all four stand caster wheels locked.' \
  'Required: clear test area and no person in reach.' \
  'Required: second operator holding the original remote controller.' \
  'Required: delivered-H2 high-level stop/damping procedure confirmed.' \
  'The axis sign is uncalibrated; do not call it forward/left yet.'
read -r -p 'Type LIVE_MOTION_PHYSICAL_GATES_CONFIRMED exactly: ' confirmation
[[ "$confirmation" == LIVE_MOTION_PHYSICAL_GATES_CONFIRMED ]] ||
  h2_die PHYSICAL_CONFIRMATION_REJECTED 50

h2_section "LAST-MOMENT GETTER RECHECK"
h2_run_getter_audit

h2_section "MOTION SWITCHER READ-ONLY CHECK"
set +e
motion_mode_output="$(
  h2_isolated timeout --foreground --signal=INT --kill-after=3s 10s \
    "$H2_BIN_DIR/robot_test_unitree_h2_motion_mode" eth0 2>&1
)"
motion_mode_rc=$?
set -e
printf '%s\n' "$motion_mode_output"
[[ "$motion_mode_rc" -eq 0 ]] ||
  h2_die "MOTION_SWITCHER_CHECK_FAILED_RC=$motion_mode_rc" 51
motion_mode_line="$(
  grep -F 'H2_MOTION_SWITCHER_CHECK ' <<<"$motion_mode_output" | tail -n 1
)"
[[ -n "$motion_mode_line" ]] || h2_die MOTION_SWITCHER_RESULT_MISSING 51
motion_switcher_form="$(
  sed -n 's/.* form=\([^ ]*\) name=.*/\1/p' <<<"$motion_mode_line"
)"
motion_switcher_name="$(
  sed -n 's/.* name=\([^ ]*\).*/\1/p' <<<"$motion_mode_line"
)"
[[ "$motion_switcher_form" == 0 ]] ||
  h2_die "MOTION_SWITCHER_FORM_UNEXPECTED=$motion_switcher_form" 51
[[ "$motion_switcher_name" == ai ]] ||
  h2_die "MOTION_SWITCHER_NAME_UNEXPECTED=$motion_switcher_name" 51
printf 'MOTION_SWITCHER_GATE_OK form=%s name=%s\n' \
  "$motion_switcher_form" "$motion_switcher_name"

printf -v final_phrase 'RUN_STREAM_%s_L%s_Y%s_S%s_H%s' \
  "$axis" "$plan_linear_speed" "$plan_yaw_speed" "$plan_stream_ms" \
  "$plan_command_hz"
printf 'FINAL_STREAM_PROFILE axis=%s linear_speed=%s yaw_speed=%s stream_ms=%s command_hz=%s command_period_ms=%s max_send_gap_ms=%s expected_rpc_count=%s vendor_duration_s=%s watchdog_ms=%s\n' \
  "$axis" "$plan_linear_speed" "$plan_yaw_speed" "$plan_stream_ms" \
  "$plan_command_hz" "$plan_command_period_ms" "$plan_max_send_gap_ms" \
  "$plan_expected_rpc_count" "$plan_vendor_duration_s" "$plan_watchdog_ms"
read -r -p "Second operator ready. Type $final_phrase exactly: " final_confirmation
[[ "$final_confirmation" == "$final_phrase" ]] ||
  h2_die FINAL_AXIS_CONFIRMATION_REJECTED 50

run_stamp="$(date +%Y%m%d_%H%M%S)_$BASHPID"
odom_phase_file="$H2_STATE_DIR/.stage06e_odom_phase_$BASHPID"
odom_csv="$H2_ROOT/logs/h2_pc2_stage06e_odom_${run_stamp}.csv"
odom_summary="$H2_ROOT/logs/h2_pc2_stage06e_odom_${run_stamp}.summary"
odom_runtime_log="$H2_ROOT/logs/h2_pc2_stage06e_odom_${run_stamp}.runtime.log"

h2_section "DOG ODOM BASELINE"
printf 'baseline\n' >"$odom_phase_file"
chmod 0600 "$odom_phase_file"
(
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source /home/unitree/graph_pid_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$H2_CYCLONE_CONFIG"
  exec python3 "$SCRIPT_DIR/h2_dog_odom_capture.py" \
    --phase-file "$odom_phase_file" \
    --csv "$odom_csv" \
    --summary "$odom_summary" \
    --sample-hz 50
) >"$odom_runtime_log" 2>&1 &
odom_pid=$!
sleep 2
kill -0 "$odom_pid" 2>/dev/null || {
  cat "$odom_runtime_log" >&2 || true
  h2_die DOG_ODOM_CAPTURE_EXITED_DURING_BASELINE 51
}
printf 'DOG_ODOM_BASELINE_CAPTURE_RUNNING pid=%s csv=%s\n' \
  "$odom_pid" "$odom_csv"

h2_section "ONE SINGLE-AXIS PULSE"
ack=H2_FALL_ARREST_STAND_FOUR_CASTERS_LOCKED_CLEAR_AREA_SECOND_OPERATOR_REMOTE_HIGH_LEVEL_STOP_CONFIRMED
authorization_token="$(od -An -N32 -tx1 /dev/urandom | tr -d ' \n')"
[[ "$authorization_token" =~ ^[0-9a-f]{64}$ ]] ||
  h2_die LIVE_AUTHORIZATION_TOKEN_GENERATION_FAILED 51
authorization_file="$H2_STATE_DIR/.stage06e_authorization_$authorization_token"
(
  umask 077
  set -o noclobber
  printf '%s\n' "$authorization_token" >"$authorization_file"
)
[[ ! -L "$authorization_file" && -f "$authorization_file" &&
   "$(stat -c '%U' "$authorization_file")" == unitree &&
   "$(stat -c '%a' "$authorization_file")" == 600 ]] ||
  h2_die LIVE_AUTHORIZATION_FILE_INVALID 51

motion_capture="$H2_STATE_DIR/.stage06e_motion.$BASHPID.log"
: >"$motion_capture"
chmod 0600 "$motion_capture"
printf 'active\n' >"$odom_phase_file"
set +e
h2_isolated env H2_LIVE_GATE_TOKEN="$authorization_token" \
  timeout --foreground --signal=INT --kill-after=3s 25s \
  "$H2_BIN_DIR/robot_test_unitree_h2_live_motion" \
  --config "$H2_CONFIG" \
  "${motion_args[@]}" \
  --expected-fsm 601 \
  --live-motion \
  --acknowledge "$ack" \
  --authorization-file "$authorization_file" 2>&1 | tee "$motion_capture"
motion_status=("${PIPESTATUS[@]}")
set -e
motion_rc="${motion_status[0]}"
[[ "${motion_status[1]}" -eq 0 ]] || h2_die MOTION_CAPTURE_TEE_FAILED 51
motion_output="$(<"$motion_capture")"
rm -f -- "$motion_capture"
motion_capture=""

authorization_consumed=0
if [[ ! -e "$authorization_file" ]]; then
  authorization_consumed=1
fi
rm -f -- "$authorization_file"
authorization_file=""

printf 'post\n' >"$odom_phase_file"
sleep 2
set +e
kill -INT "$odom_pid" 2>/dev/null
kill_rc=$?
wait "$odom_pid"
odom_rc=$?
set -e
odom_pid=""
[[ "$kill_rc" -eq 0 ]] || h2_die DOG_ODOM_CAPTURE_SIGNAL_FAILED 52
[[ "$odom_rc" -eq 0 ]] || {
  cat "$odom_runtime_log" >&2 || true
  h2_die "DOG_ODOM_CAPTURE_FAILED_RC=$odom_rc" 52
}
[[ -f "$odom_summary" ]] || h2_die DOG_ODOM_SUMMARY_MISSING 52

h2_section "DOG ODOM SUMMARY"
cat "$odom_summary"

extract_odom_field() {
  local key="$1"
  local count value
  count="$(grep -cE "^${key}=" "$odom_summary" || true)"
  [[ "$count" == 1 ]] || h2_die "DOG_ODOM_FIELD_COUNT_${key}=$count" 52
  value="$(sed -n "s/^${key}=//p" "$odom_summary")"
  [[ -n "$value" ]] || h2_die "DOG_ODOM_FIELD_EMPTY=$key" 52
  printf '%s\n' "$value"
}

odom_topic="$(extract_odom_field odom_topic)"
odom_sample_count="$(extract_odom_field odom_sample_count)"
odom_baseline_sample_count="$(extract_odom_field odom_baseline_sample_count)"
odom_active_sample_count="$(extract_odom_field odom_active_sample_count)"
odom_post_sample_count="$(extract_odom_field odom_post_sample_count)"
odom_baseline_max_planar_speed="$(extract_odom_field odom_baseline_max_planar_speed)"
odom_active_max_planar_speed="$(extract_odom_field odom_active_max_planar_speed)"
odom_post_max_planar_speed="$(extract_odom_field odom_post_max_planar_speed)"
odom_active_delta_xy="$(extract_odom_field odom_active_delta_xy)"
odom_effective_speed_threshold="$(extract_odom_field odom_effective_speed_threshold)"
odom_speed_response="$(extract_odom_field odom_speed_response)"
odom_position_response="$(extract_odom_field odom_position_response)"
odom_capture_ok="$(extract_odom_field odom_capture_ok)"
odom_response_detected="$(extract_odom_field odom_response_detected)"

[[ "$odom_topic" == /dog_odom ]] || h2_die DOG_ODOM_TOPIC_MISMATCH 52
[[ "$odom_capture_ok" == 1 ]] || h2_die DOG_ODOM_CAPTURE_INCOMPLETE 52
for integer_value in "$odom_sample_count" "$odom_baseline_sample_count" \
  "$odom_active_sample_count" "$odom_post_sample_count"; do
  [[ "$integer_value" =~ ^[0-9]+$ ]] || h2_die DOG_ODOM_COUNT_INVALID 52
done
for flag_value in "$odom_speed_response" "$odom_position_response" \
  "$odom_capture_ok" "$odom_response_detected"; do
  [[ "$flag_value" == 0 || "$flag_value" == 1 ]] ||
    h2_die DOG_ODOM_FLAG_INVALID 52
done
for float_value in "$odom_baseline_max_planar_speed" \
  "$odom_active_max_planar_speed" "$odom_post_max_planar_speed" \
  "$odom_active_delta_xy" "$odom_effective_speed_threshold"; do
  [[ "$float_value" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    h2_die DOG_ODOM_FLOAT_INVALID 52
done

printf 'COMMAND_RC[single_axis_motion]=%s\n' "$motion_rc"
[[ "$authorization_consumed" -eq 1 ]] ||
  h2_die LIVE_AUTHORIZATION_FILE_NOT_CONSUMED 51
[[ "$motion_rc" -eq 0 ]] ||
  h2_die "SINGLE_AXIS_MOTION_FAILED_RC=$motion_rc" 51

stream_success_line="$(
  grep -F "H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK axis=$axis" \
    <<<"$motion_output" | tail -n 1
)"
[[ -n "$stream_success_line" ]] ||
  h2_die SINGLE_AXIS_STREAM_SUCCESS_MARKER_MISSING 51

extract_stream_success_field() {
  local key="$1"
  if [[ "$stream_success_line" =~ (^|[[:space:]])${key}=([^[:space:]]+) ]]; then
    printf '%s\n' "${BASH_REMATCH[2]}"
    return 0
  fi
  h2_die "STREAM_SUCCESS_FIELD_MISSING=$key" 51
}

stream_rpc_count="$(extract_stream_success_field rpc_count)"
stream_marker_expected_rpc_count="$(
  extract_stream_success_field expected_rpc_count
)"
stream_max_observed_send_gap_ms="$(
  extract_stream_success_field max_observed_send_gap_ms
)"
[[ "$stream_marker_expected_rpc_count" == "$plan_expected_rpc_count" ]] ||
  h2_die STREAM_MARKER_EXPECTED_RPC_COUNT_MISMATCH 51
h2_validate_stage06e_stream_profile \
  "$plan_linear_speed" "$plan_yaw_speed" "$plan_stream_ms" \
  "$plan_command_hz" "$plan_command_period_ms" "$plan_max_send_gap_ms" \
  "$plan_expected_rpc_count" "$stream_rpc_count" \
  "$plan_vendor_duration_s" "$plan_watchdog_ms" \
  "$stream_max_observed_send_gap_ms"

h2_section "POST-PULSE GETTER RECHECK"
h2_run_getter_audit
h2_check_no_probe_process

read -r -p \
  'Type PHYSICAL_MOTION_OBSERVED or NO_PHYSICAL_MOTION_OBSERVED exactly: ' \
  physical_result
case "$physical_result" in
  PHYSICAL_MOTION_OBSERVED)
    physical_motion_observed=1
    ;;
  NO_PHYSICAL_MOTION_OBSERVED)
    physical_motion_observed=0
    ;;
  *)
    h2_die INVALID_PHYSICAL_MOTION_RESULT 52
    ;;
esac

if [[ "$physical_motion_observed" -eq 0 &&
      "$odom_response_detected" -eq 0 ]]; then
  h2_die COMMAND_ACCEPTED_BUT_ODOM_UNCHANGED 52
fi
if [[ "$physical_motion_observed" -eq 0 &&
      "$odom_response_detected" -eq 1 ]]; then
  h2_die ODOM_CHANGED_BUT_PHYSICAL_MOTION_NOT_CONFIRMED 52
fi
if [[ "$physical_motion_observed" -eq 1 &&
      "$odom_response_detected" -eq 0 ]]; then
  h2_die PHYSICAL_MOTION_WITHOUT_ODOM_CONFIRMATION 52
fi

read -r -p 'Record the observed physical direction: ' observed
[[ -n "$observed" ]] || h2_die OBSERVED_DIRECTION_REQUIRED 52
observed_clean="$(printf '%s' "$observed" | tr '\t\r\n' '   ')"
if h2_observation_reports_no_motion "$observed_clean"; then
  h2_die PHYSICAL_MOTION_DIRECTION_REPORTS_NO_MOTION 52
fi

stream_observation_log="$H2_ROOT/logs/h2_pc2_axis_stream_observations_r9.tsv"
if [[ ! -e "$stream_observation_log" ]]; then
  printf 'timestamp\taxis\tlinear_speed\tyaw_speed\tstream_ms\tcommand_hz\trpc_count\tmotion_switcher_form\tmotion_switcher_name\todom_sample_count\todom_baseline_max_planar_speed\todom_active_max_planar_speed\todom_post_max_planar_speed\todom_active_delta_xy\todom_effective_speed_threshold\todom_speed_response\todom_position_response\todom_response_detected\tphysical_motion_observed\tobservation\tfsm_id\tboot_id\tmanifest_sha256\todom_csv\todom_summary\n' \
    >"$stream_observation_log"
fi
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
  "$(date --iso-8601=seconds)" "$axis" "$plan_linear_speed" \
  "$plan_yaw_speed" "$plan_stream_ms" "$plan_command_hz" \
  "$stream_rpc_count" "$motion_switcher_form" "$motion_switcher_name" \
  "$odom_sample_count" "$odom_baseline_max_planar_speed" \
  "$odom_active_max_planar_speed" "$odom_post_max_planar_speed" \
  "$odom_active_delta_xy" "$odom_effective_speed_threshold" \
  "$odom_speed_response" "$odom_position_response" \
  "$odom_response_detected" "$physical_motion_observed" "$observed_clean" \
  "$H2_FSM_ID" "$H2_BOOT_ID" "$H2_MANIFEST_SHA256" \
  "$odom_csv" "$odom_summary" >>"$stream_observation_log"

read -r -p 'Type BOUNDED_STREAM_OBSERVED_SAFE exactly: ' safety_observation
[[ "$safety_observation" == BOUNDED_STREAM_OBSERVED_SAFE ]] ||
  h2_die MOTION_OBSERVATION_NOT_ACCEPTED 52

# The durable r9 odom evidence is stored in the TSV/CSV/summary logs above.
# Keep the existing Stage 06E gate schema compatible with Stage 06C/06D tools.
h2_write_gate 06e \
  "axis=$axis" \
  "linear_speed=$plan_linear_speed" \
  "yaw_speed=$plan_yaw_speed" \
  "stream_ms=$plan_stream_ms" \
  "command_hz=$plan_command_hz" \
  "command_period_ms=$plan_command_period_ms" \
  "max_send_gap_ms=$plan_max_send_gap_ms" \
  "expected_rpc_count=$plan_expected_rpc_count" \
  "rpc_count=$stream_rpc_count" \
  "max_observed_send_gap_ms=$stream_max_observed_send_gap_ms" \
  "vendor_duration_s=$plan_vendor_duration_s" \
  "watchdog_ms=$plan_watchdog_ms" \
  "observed_physical_direction=$observed_clean" \
  "fsm_id=$H2_FSM_ID" \
  "observer=BOUNDED_STREAM_OBSERVED_SAFE" \
  "state_changing_action_invoked=0" \
  "single_axis_stream_rpc_ok=1" \
  "parent_stage06c_sha256=$stage06c_gate_sha256" \
  "parent_stage06d_sha256=$stage06d_gate_sha256"

printf 'H2_STAGE06E_SINGLE_AXIS_STREAM_OK axis=%s linear_speed=%s yaw_speed=%s stream_ms=%s command_hz=%s rpc_count=%s odom_response_detected=%s physical_motion_observed=%s observed=%s\n' \
  "$axis" "$plan_linear_speed" "$plan_yaw_speed" \
  "$plan_stream_ms" "$plan_command_hz" "$stream_rpc_count" \
  "$odom_response_detected" "$physical_motion_observed" "$observed_clean"
