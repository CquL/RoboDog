#!/usr/bin/env bash

# Stage 06C: getter-only H2 Loco RPC compatibility gate.
# This sends DDS RPC requests to /api/sport/request, but invokes no velocity,
# StopMove, state-changing action, low-level command, or ROS control topic.

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd -P)"
# shellcheck source=h2_pc2_hal_gate_common.sh
source "$SCRIPT_DIR/h2_pc2_hal_gate_common.sh"

h2_prepare_gate 06c
h2_run_offline_contracts

h2_section "GETTER-ONLY LOCO RPC"
h2_run_getter_audit
h2_check_no_probe_process

h2_write_gate 06c \
  "fsm_id=$H2_FSM_ID" \
  "fsm_mode=$H2_FSM_MODE" \
  "mode_value_observation_only=1" \
  "control_setter_invoked=0" \
  "getter_only_rpc_ok=1"

printf 'H2_STAGE06C_GETTER_RPC_OK fsm_id=%s fsm_mode=%s\n' \
  "$H2_FSM_ID" "$H2_FSM_MODE"
