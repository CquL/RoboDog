#!/usr/bin/env bash

# Stage 06D: protected zero velocity plus StopMove write-RPC gate.
# StopMove is a high-level SetVelocity(0,0,0) wrapper, not a hardware E-stop.

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/h2_pc2_hal_gate_common.sh"

zero_capture=""
cleanup_zero_capture() {
  if [[ -n "$zero_capture" ]]; then
    rm -f -- "$zero_capture"
  fi
}
trap cleanup_zero_capture EXIT

h2_prepare_gate 06d
h2_require_gate 06c \
  "fsm_id=601" \
  "mode_value_observation_only=1" \
  "control_setter_invoked=0" \
  "getter_only_rpc_ok=1"
stage06c_gate_sha256="$(
  sha256sum "$H2_STATE_DIR/stage06c.ok" | awk '{print $1}'
)"
h2_run_offline_contracts

# h2_prepare_gate redirects stdout through tee; FD 3 remains the original TTY.
[[ -t 0 && -t 3 ]] || h2_die STAGE06D_REQUIRES_INTERACTIVE_TTY 40
h2_section "PHYSICAL ZERO-STOP CHECKLIST"
printf '%s\n' \
  'Required: official fall-arrest/protective stand installed.' \
  'Required: all four stand caster wheels locked.' \
  'Required: clear test area and no person in reach.' \
  'Required: second operator holding the original remote controller.' \
  'Required: delivered-H2 high-level stop/damping procedure confirmed.'
read -r -p 'Type ZERO_STOP_PHYSICAL_GATES_CONFIRMED exactly: ' confirmation
[[ "$confirmation" == ZERO_STOP_PHYSICAL_GATES_CONFIRMED ]] ||
  h2_die PHYSICAL_CONFIRMATION_REJECTED 40

h2_section "PRE-WRITE GETTER RECHECK"
h2_run_getter_audit

h2_section "ZERO VELOCITY AND STOPMOVE"
zero_capture="$H2_STATE_DIR/.stage06d_zero_stop.$BASHPID.log"
: >"$zero_capture"
chmod 0600 "$zero_capture"
set +e
h2_isolated timeout --foreground --signal=INT --kill-after=3s 25s \
  "$H2_BIN_DIR/robot_test_unitree_h2" \
  "$H2_CONFIG" --zero-stop 2>&1 | tee "$zero_capture"
zero_status=("${PIPESTATUS[@]}")
set -e
zero_rc="${zero_status[0]}"
[[ "${zero_status[1]}" -eq 0 ]] || h2_die ZERO_STOP_CAPTURE_TEE_FAILED 41
zero_output="$(<"$zero_capture")"
rm -f -- "$zero_capture"
zero_capture=""
printf 'COMMAND_RC[zero_stop]=%s\n' "$zero_rc"
[[ "$zero_rc" -eq 0 ]] || h2_die "ZERO_STOP_COMMAND_FAILED_RC=$zero_rc" 41
grep -F 'H2_ZERO_STOP_RPC_OK' <<<"$zero_output" >/dev/null ||
  h2_die ZERO_STOP_SUCCESS_MARKER_MISSING 41

h2_section "POST-WRITE GETTER RECHECK"
h2_run_getter_audit
h2_check_no_probe_process

read -r -p 'Observer: type NO_UNEXPECTED_MOTION_OBSERVED exactly: ' observation
[[ "$observation" == NO_UNEXPECTED_MOTION_OBSERVED ]] ||
  h2_die ZERO_STOP_OBSERVATION_NOT_ACCEPTED 42

h2_write_gate 06d \
  "fsm_id=$H2_FSM_ID" \
  "fsm_mode=$H2_FSM_MODE" \
  "observer=NO_UNEXPECTED_MOTION_OBSERVED" \
  "nonzero_velocity_invoked=0" \
  "zero_stop_rpc_ok=1" \
  "parent_stage06c_sha256=$stage06c_gate_sha256"

printf 'H2_STAGE06D_ZERO_STOP_OBSERVED_OK fsm_id=%s\n' "$H2_FSM_ID"
