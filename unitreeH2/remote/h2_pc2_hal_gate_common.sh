#!/usr/bin/env bash

# Fail-closed checks shared by the H2 PC2 Stage 06C/06D/06E runners.
# Sourcing this file never calls a robot API.

set -Eeuo pipefail

H2_ROOT=/home/unitree/p2_unitreeH2
H2_CYCLONE_CONFIG=/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml
H2_EXPECTED_CYCLONE_SHA256=bc977aacd0e44804cb8da24d24d6fe5ed654aad9ed7b15ed3ef36d32f27a1796
H2_EXPECTED_DDSC_SHA256=4038630c231412f7b34a2ea60df192bbdebd0a57f22ac7f35c1b6d28323e695c
H2_EXPECTED_DDSCXX_SHA256=d3e7c1b03123c2745839f2465041777ded090ad66d62f1372949209254f7ebe5
H2_EXPECTED_HOSTNAME=unitree-H2-pc2
H2_EXPECTED_PC2_IPV4=192.168.123.162/24
H2_EXPECTED_PC1_IPV4=192.168.123.161

H2_COMMON_DIR="$(cd -- "$(dirname -- "$BASH_SOURCE")" && pwd -P)"
H2_RELEASE="$(cd -- "$H2_COMMON_DIR/.." && pwd -P)"
H2_BIN_DIR="$H2_RELEASE/bin"
H2_LIB_DIR="$H2_RELEASE/lib"
H2_CONFIG="$H2_RELEASE/config/unitree_h2.yaml"
H2_MANIFEST="$H2_RELEASE/meta/manifest.sha256"
H2_STATE_DIR="$H2_ROOT/build/h2_control_gate_state"
H2_LOCK_FILE="$H2_STATE_DIR/h2_hal_control.lock"

h2_section() {
  printf '\n===== %s =====\n' "$1"
}

h2_die() {
  local code=1
  if [[ $# -ge 2 ]]; then code="$2"; fi
  printf 'H2_GATE_FAILED=%s\n' "$1" >&2
  exit "$code"
}

h2_normalize_motion_observation() {
  local observation="$1"
  printf '%s' "$observation" |
    tr '[:lower:]' '[:upper:]' |
    sed -E 's/[[:space:]-]+/_/g; s/^_+//; s/_+$//'
}

h2_observation_reports_no_motion() {
  local normalized
  normalized="$(h2_normalize_motion_observation "$1")"
  case "$normalized" in
    NO_MOTION|NO_MOTION_OBSERVED|NO_VISIBLE_MOTION|NO_VISIBLE_MOTION_OBSERVED|\
    NO_PHYSICAL_MOTION|NO_PHYSICAL_MOTION_OBSERVED|NO_MOVEMENT|\
    NO_MOVEMENT_OBSERVED|NO_VISIBLE_MOVEMENT|NO_VISIBLE_MOVEMENT_OBSERVED|\
    NO_PHYSICAL_MOVEMENT|NO_PHYSICAL_MOVEMENT_OBSERVED)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

h2_validate_stage06e_stream_profile() {
  local linear_speed="$1"
  local yaw_speed="$2"
  local stream_ms="$3"
  local command_hz="$4"
  local command_period_ms="$5"
  local max_send_gap_ms="$6"
  local expected_rpc_count="$7"
  local rpc_count="$8"
  local vendor_duration_s="$9"
  local watchdog_ms="${10}"
  local max_observed_send_gap_ms="${11}"

  [[ "$linear_speed" =~ ^0\.[0-9]+$ ]] ||
    h2_die STAGE-06e-LINEAR-SPEED-INVALID 19
  [[ "$yaw_speed" =~ ^0\.[0-9]+$ ]] ||
    h2_die STAGE-06e-YAW-SPEED-INVALID 19
  local integer_value
  for integer_value in "$stream_ms" "$command_hz" "$command_period_ms" \
    "$max_send_gap_ms" "$expected_rpc_count" "$rpc_count" \
    "$watchdog_ms" "$max_observed_send_gap_ms"; do
    [[ "$integer_value" =~ ^[0-9]+$ ]] ||
      h2_die STAGE-06e-STREAM-INTEGER-FIELD-INVALID 19
  done
  [[ "$vendor_duration_s" == 0.300 ]] ||
    h2_die STAGE-06e-VENDOR-DURATION-INVALID 19

  awk -v value="$linear_speed" \
    'BEGIN { exit !(value >= 0.01 && value <= 0.10) }' ||
    h2_die STAGE-06e-LINEAR-SPEED-OUT-OF-RANGE 19
  awk -v value="$yaw_speed" \
    'BEGIN { exit !(value >= 0.01 && value <= 0.15) }' ||
    h2_die STAGE-06e-YAW-SPEED-OUT-OF-RANGE 19
  (( stream_ms >= 250 && stream_ms <= 1000 && stream_ms % 50 == 0 )) ||
    h2_die STAGE-06e-STREAM-MS-OUT-OF-RANGE-OR-STEP 19
  (( command_hz == 20 )) || h2_die STAGE-06e-COMMAND-HZ-INVALID 19
  (( command_period_ms == 50 )) ||
    h2_die STAGE-06e-COMMAND-PERIOD-MS-INVALID 19
  (( max_send_gap_ms == 100 )) ||
    h2_die STAGE-06e-MAX-SEND-GAP-MS-INVALID 19
  (( watchdog_ms == 150 )) || h2_die STAGE-06e-WATCHDOG-MS-INVALID 19
  (( expected_rpc_count == stream_ms / command_period_ms )) ||
    h2_die STAGE-06e-EXPECTED-RPC-COUNT-INVALID 19
  (( rpc_count == expected_rpc_count )) ||
    h2_die STAGE-06e-RPC-COUNT-MISMATCH 19
  (( max_observed_send_gap_ms <= max_send_gap_ms )) ||
    h2_die STAGE-06e-OBSERVED-SEND-GAP-EXCEEDED 19
}

h2_verify_sha256() {
  printf '%s  %s\n' "$1" "$2" | sha256sum --check --strict -
}

h2_isolated() {
  env -i \
    HOME=/home/unitree \
    USER=unitree \
    LOGNAME=unitree \
    LANG=C.UTF-8 \
    PATH=/usr/bin:/bin \
    LD_LIBRARY_PATH="$H2_LIB_DIR" \
    CYCLONEDDS_URI="$H2_CYCLONE_CONFIG" \
    "$@"
}

h2_check_no_probe_process() {
  if pgrep -af '[r]obot_test_unitree_h2'; then
    h2_die PREEXISTING_H2_HAL_PROBE_REFUSED 18
  fi
}

h2_validate_gate_schema() {
  local stage="$1"
  local gate="$2"
  local -a expected_keys gate_lines
  case "$stage" in
    06c)
      expected_keys=(stage timestamp hostname boot_id release manifest_sha256 log \
        fsm_id fsm_mode mode_value_observation_only control_setter_invoked \
        getter_only_rpc_ok)
      ;;
    06d)
      expected_keys=(stage timestamp hostname boot_id release manifest_sha256 log \
        fsm_id fsm_mode observer nonzero_velocity_invoked zero_stop_rpc_ok \
        parent_stage06c_sha256)
      ;;
    06e)
      expected_keys=(stage timestamp hostname boot_id release manifest_sha256 log \
        axis linear_speed yaw_speed stream_ms command_hz command_period_ms \
        max_send_gap_ms expected_rpc_count rpc_count max_observed_send_gap_ms \
        vendor_duration_s watchdog_ms observed_physical_direction fsm_id observer \
        state_changing_action_invoked single_axis_stream_rpc_ok \
        parent_stage06c_sha256 parent_stage06d_sha256)
      ;;
    *) h2_die "UNKNOWN-GATE-SCHEMA-STAGE=$stage" 19 ;;
  esac

  mapfile -t gate_lines <"$gate"
  [[ "${#gate_lines[@]}" -eq "${#expected_keys[@]}" ]] ||
    h2_die "STAGE-$stage-GATE-LINE-COUNT=${#gate_lines[@]}" 19
  local index key
  for index in "${!expected_keys[@]}"; do
    key="${gate_lines[$index]%%=*}"
    [[ "$key" == "${expected_keys[$index]}" ]] ||
      h2_die "STAGE-$stage-GATE-KEY-$index=$key" 19
  done
  [[ "${gate_lines[1]}" =~ ^timestamp=.+$ ]] ||
    h2_die "STAGE-$stage-TIMESTAMP-INVALID" 19
  [[ "${gate_lines[6]}" =~ ^log=$H2_ROOT/logs/h2_pc2_stage${stage}_.+\.log$ ]] ||
    h2_die "STAGE-$stage-LOG-PATH-INVALID" 19
  if [[ "$stage" == 06c || "$stage" == 06d ]]; then
    [[ "${gate_lines[8]}" =~ ^fsm_mode=-?[0-9]+$ ]] ||
      h2_die "STAGE-$stage-FSM-MODE-INVALID" 19
  fi
  if [[ "$stage" == 06e ]]; then
    [[ "${gate_lines[7]}" =~ ^axis=(x|y|yaw)-(positive|negative)$ ]] ||
      h2_die STAGE-06e-AXIS-INVALID 19
    [[ "${gate_lines[19]}" =~ ^observed_physical_direction=.+$ ]] ||
      h2_die STAGE-06e-OBSERVED-DIRECTION-INVALID 19
    local observed_physical_direction="${gate_lines[19]#*=}"
    if h2_observation_reports_no_motion "$observed_physical_direction"; then
      h2_die STAGE-06e-NO-PHYSICAL-MOTION-CANNOT-BE-SUCCESS-GATE 19
    fi
    [[ "${gate_lines[20]}" == 'fsm_id=601' ]] ||
      h2_die STAGE-06e-FSM-ID-INVALID 19
    [[ "${gate_lines[21]}" == 'observer=BOUNDED_STREAM_OBSERVED_SAFE' ]] ||
      h2_die STAGE-06e-OBSERVER-INVALID 19
    [[ "${gate_lines[22]}" == 'state_changing_action_invoked=0' ]] ||
      h2_die STAGE-06e-STATE-CHANGING-ACTION-FLAG-INVALID 19
    [[ "${gate_lines[23]}" == 'single_axis_stream_rpc_ok=1' ]] ||
      h2_die STAGE-06e-SINGLE-AXIS-STREAM-FLAG-INVALID 19
    local linear_speed="${gate_lines[8]#*=}"
    local yaw_speed="${gate_lines[9]#*=}"
    local stream_ms="${gate_lines[10]#*=}"
    local command_hz="${gate_lines[11]#*=}"
    local command_period_ms="${gate_lines[12]#*=}"
    local max_send_gap_ms="${gate_lines[13]#*=}"
    local expected_rpc_count="${gate_lines[14]#*=}"
    local rpc_count="${gate_lines[15]#*=}"
    local max_observed_send_gap_ms="${gate_lines[16]#*=}"
    local vendor_duration_s="${gate_lines[17]#*=}"
    local watchdog_ms="${gate_lines[18]#*=}"
    h2_validate_stage06e_stream_profile \
      "$linear_speed" "$yaw_speed" "$stream_ms" "$command_hz" \
      "$command_period_ms" "$max_send_gap_ms" "$expected_rpc_count" \
      "$rpc_count" "$vendor_duration_s" "$watchdog_ms" \
      "$max_observed_send_gap_ms"
  fi
}

h2_require_gate() {
  local stage="$1"
  shift
  local gate="$H2_STATE_DIR/stage$stage.ok"
  [[ ! -L "$gate" && -f "$gate" && -r "$gate" ]] ||
    h2_die "MISSING-STAGE-$stage-GATE=$gate" 19
  [[ "$(stat -c '%U' "$gate")" == unitree &&
     "$(stat -c '%a' "$gate")" == 600 ]] ||
    h2_die "STAGE-$stage-GATE-OWNER-OR-MODE-INVALID" 19
  h2_validate_gate_schema "$stage" "$gate"

  local field count
  for field in \
    "stage=$stage" \
    "hostname=$H2_EXPECTED_HOSTNAME" \
    "manifest_sha256=$H2_MANIFEST_SHA256" \
    "boot_id=$H2_BOOT_ID" \
    "release=$H2_RELEASE" \
    "$@"; do
    count="$(grep -Fxc -- "$field" "$gate" || true)"
    [[ "$count" == 1 ]] ||
      h2_die "STAGE-$stage-REQUIRED-FIELD-COUNT-$count=$field" 19
  done
  printf 'REQUIRED_GATE_OK=stage%s\n' "$stage"
  cat "$gate"
}

h2_write_gate() {
  local stage="$1"
  shift
  local gate="$H2_STATE_DIR/stage$stage.ok"
  local temporary="$gate.tmp.$BASHPID"
  {
    printf 'stage=%s\n' "$stage"
    printf 'timestamp=%s\n' "$(date --iso-8601=seconds)"
    printf 'hostname=%s\n' "$(hostname)"
    printf 'boot_id=%s\n' "$H2_BOOT_ID"
    printf 'release=%s\n' "$H2_RELEASE"
    printf 'manifest_sha256=%s\n' "$H2_MANIFEST_SHA256"
    printf 'log=%s\n' "$H2_LOG"
    local field
    for field in "$@"; do printf '%s\n' "$field"; done
  } >"$temporary"
  chmod 0600 "$temporary"
  mv -f "$temporary" "$gate"
  case "$stage" in
    06c)
      rm -f -- "$H2_STATE_DIR/stage06d.ok" "$H2_STATE_DIR/stage06e.ok"
      ;;
    06d)
      rm -f -- "$H2_STATE_DIR/stage06e.ok"
      ;;
  esac
  printf 'GATE_WRITTEN=%s\n' "$gate"
}

h2_run_offline_contracts() {
  h2_section "OFFLINE CONTRACT TESTS"
  h2_isolated "$H2_BIN_DIR/unitree_h2_factory_contract_test"
  h2_isolated "$H2_BIN_DIR/unitree_h2_direct_api_contract_test"
  h2_isolated "$H2_BIN_DIR/unitree_h2_live_motion_plan_test"
  printf 'H2_PC2_BUNDLE_OFFLINE_CONTRACTS_OK\n'
}

h2_run_getter_audit() {
  local output rc
  set +e
  output="$(
    h2_isolated timeout --foreground --signal=INT --kill-after=3s 25s \
      "$H2_BIN_DIR/robot_test_unitree_h2" \
      --config "$H2_CONFIG" --getter-audit 2>&1
  )"
  rc=$?
  set -e
  printf '%s\n' "$output"
  printf 'COMMAND_RC[getter_audit]=%s\n' "$rc"
  [[ "$rc" -eq 0 ]] || h2_die "GETTER_AUDIT_COMMAND_FAILED_RC=$rc" 30
  grep -F H2_GETTER_ONLY_RPC_OK <<<"$output" >/dev/null ||
    h2_die GETTER_AUDIT_SUCCESS_MARKER_MISSING 30

  H2_FSM_ID="$(
    sed -n 's/.*H2_GETTER_AUDIT fsm_id=\([-0-9][0-9]*\).*/\1/p' \
      <<<"$output" | tail -n 1
  )"
  H2_FSM_MODE="$(
    sed -n 's/.*fsm_mode=\([-0-9][0-9]*\).*/\1/p' \
      <<<"$output" | tail -n 1
  )"
  [[ -n "$H2_FSM_ID" && -n "$H2_FSM_MODE" ]] ||
    h2_die GETTER_AUDIT_STATE_PARSE_FAILED 30
  [[ "$H2_FSM_ID" == 601 ]] ||
    h2_die CURRENT_FSM_IS_NOT_601_NO_AUTOMATIC_TRANSITION 31
  grep -Eq '\[601(:|])' <<<"$output" ||
    h2_die AVAILABLE_FSM_LIST_DOES_NOT_CONTAIN_601 31
  printf 'GETTER_AUDIT_GATE_OK fsm_id=%s fsm_mode=%s mode_value_observation_only=1\n' \
    "$H2_FSM_ID" "$H2_FSM_MODE"
}

h2_prepare_gate() {
  local stage="$1"
  local run_id
  run_id="$(date +%Y%m%d_%H%M%S)_$BASHPID"

  case "$H2_RELEASE/" in
    "$H2_ROOT"/build/*/) ;;
    *) h2_die "RELEASE_OUTSIDE_APPROVED_PC2_BUILD_ROOT=$H2_RELEASE" 10 ;;
  esac

  umask 077
  mkdir -p "$H2_ROOT/logs" "$H2_STATE_DIR"
  [[ ! -L "$H2_STATE_DIR" && -d "$H2_STATE_DIR" ]] ||
    h2_die CONTROL_GATE_STATE_DIR_INVALID 10
  chmod 0700 "$H2_STATE_DIR"
  printf -v H2_LOG '%s/logs/h2_pc2_stage%s_%s.log' \
    "$H2_ROOT" "$stage" "$run_id"
  export H2_LOG
  # Keep the caller's original stdout before tee changes stdout into a pipe.
  # Stage 06D/06E use FD 3 with stdin to validate real human interaction.
  exec 3>&1
  exec > >(tee -a "$H2_LOG") 2>&1

  h2_section IDENTITY
  printf 'STAGE=%s\nRELEASE=%s\nLOG=%s\n' "$stage" "$H2_RELEASE" "$H2_LOG"
  printf 'DATE=%s\nHOSTNAME=%s\nUSER=%s\nARCH=%s\n' \
    "$(date --iso-8601=seconds)" "$(hostname)" "$(id -un)" "$(uname -m)"

  local tool
  for tool in bash sha256sum ldd file readelf strings grep sed tail ip ping \
    timeout flock pgrep readlink stat find hostname tee awk tr od rm chmod; do
    command -v "$tool" >/dev/null 2>&1 || h2_die "MISSING_TOOL=$tool" 11
  done

  [[ "$(id -un)" == unitree ]] || h2_die MUST_RUN_AS_UNITREE_USER 12
  [[ "$(hostname)" == "$H2_EXPECTED_HOSTNAME" ]] ||
    h2_die "UNEXPECTED_HOSTNAME=$(hostname)" 12
  [[ "$(uname -m)" == x86_64 ]] || h2_die "UNEXPECTED_ARCH=$(uname -m)" 12
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] ||
    h2_die "UNEXPECTED_OS=$ID-$VERSION_ID" 12

  H2_BOOT_ID="$(tr -d '\r\n' </proc/sys/kernel/random/boot_id)"
  export H2_BOOT_ID
  printf 'BOOT_ID=%s\n' "$H2_BOOT_ID"

  exec 9>"$H2_LOCK_FILE"
  flock -n 9 || h2_die ANOTHER_H2_HAL_GATE_IS_RUNNING 13
  printf 'CONTROL_LOCK_OK=%s\n' "$H2_LOCK_FILE"

  h2_section "NETWORK AND VENDOR DDS CONFIG"
  ip -o link show dev eth0 | grep -qE '<[^>]*(UP|LOWER_UP)[^>]*>' ||
    h2_die ETH0_NOT_UP 14
  ip -o -4 addr show dev eth0 | grep -F "$H2_EXPECTED_PC2_IPV4" >/dev/null ||
    h2_die ETH0_EXPECTED_ADDRESS_MISSING 14
  ping -c 1 -W 1 "$H2_EXPECTED_PC1_IPV4" >/dev/null ||
    h2_die "PC1_NOT_REACHABLE=$H2_EXPECTED_PC1_IPV4" 14
  [[ -f "$H2_CYCLONE_CONFIG" && -r "$H2_CYCLONE_CONFIG" ]] ||
    h2_die CYCLONEDDS_CONFIG_NOT_READABLE 14
  h2_verify_sha256 "$H2_EXPECTED_CYCLONE_SHA256" "$H2_CYCLONE_CONFIG"
  printf 'ETH0_AND_PC1_REACHABILITY_OK\n'
  pgrep -af 'sport_switch|dog_cmd' || true

  h2_section "BUNDLE MANIFEST AND OWNERSHIP"
  [[ -f "$H2_MANIFEST" && -r "$H2_MANIFEST" ]] ||
    h2_die "BUNDLE_MANIFEST_MISSING=$H2_MANIFEST" 15
  H2_MANIFEST_SHA256="$(sha256sum "$H2_MANIFEST" | awk '{print $1}')"
  export H2_MANIFEST_SHA256
  (cd "$H2_RELEASE" && sha256sum --check --strict meta/manifest.sha256)
  if find "$H2_RELEASE" -xdev -type f -perm -0002 -print -quit | grep -q .; then
    h2_die WORLD_WRITABLE_BUNDLE_FILE_REFUSED 15
  fi
  if find "$H2_RELEASE" -xdev ! -user unitree -print -quit | grep -q .; then
    h2_die BUNDLE_MUST_BE_EXTRACTED_BY_UNITREE_USER 15
  fi
  printf 'MANIFEST_SHA256=%s\n' "$H2_MANIFEST_SHA256"

  h2_section "PRIVATE LIBRARY CONTRACT"
  [[ "$(readlink "$H2_LIB_DIR/libddsc.so.0")" == libddsc.so ]] ||
    h2_die DDSC_SONAME_LINK_INVALID 16
  [[ "$(readlink "$H2_LIB_DIR/libddscxx.so.0")" == libddscxx.so ]] ||
    h2_die DDSCXX_SONAME_LINK_INVALID 16
  [[ "$(readlink "$H2_LIB_DIR/libyaml-cpp.so.0.7")" == \
      libyaml-cpp.so.0.7.0 ]] || h2_die YAML_CPP_SONAME_LINK_INVALID 16
  h2_verify_sha256 "$H2_EXPECTED_DDSC_SHA256" "$H2_LIB_DIR/libddsc.so"
  h2_verify_sha256 "$H2_EXPECTED_DDSCXX_SHA256" "$H2_LIB_DIR/libddscxx.so"

  local binary ldd_output
  for binary in robot_test_unitree_h2; do
    [[ -x "$H2_BIN_DIR/$binary" ]] || h2_die "MISSING_EXECUTABLE=$binary" 16
    file "$H2_BIN_DIR/$binary"
    readelf -h "$H2_BIN_DIR/$binary" | grep -E 'Class:|Machine:'
    ldd_output="$(LD_LIBRARY_PATH="$H2_LIB_DIR" ldd "$H2_BIN_DIR/$binary")"
    printf '%s\n' "$ldd_output"
    grep -q 'not found' <<<"$ldd_output" &&
      h2_die "MISSING_DYNAMIC_LIBRARY=$binary" 16
    grep -F "librobot_hardware.so => $H2_LIB_DIR/librobot_hardware.so" \
      <<<"$ldd_output" >/dev/null ||
      h2_die ROBOT_HARDWARE_NOT_RESOLVED_TO_PRIVATE_BUNDLE 16
    grep -F "libyaml-cpp.so.0.7 => $H2_LIB_DIR/libyaml-cpp.so.0.7" \
      <<<"$ldd_output" >/dev/null ||
      h2_die YAML_CPP_NOT_RESOLVED_TO_PRIVATE_BUNDLE 16
    grep -F "libmc_sdk_zsl_1_x86_64.so => $H2_LIB_DIR/libmc_sdk_zsl_1_x86_64.so" \
      <<<"$ldd_output" >/dev/null ||
      h2_die ZSIBOT_NOT_RESOLVED_TO_PRIVATE_BUNDLE 16
    grep -F "libddsc.so.0 => $H2_LIB_DIR/libddsc.so.0" \
      <<<"$ldd_output" >/dev/null ||
      h2_die DDSC_NOT_RESOLVED_TO_PRIVATE_BUNDLE 16
    grep -F "libddscxx.so.0 => $H2_LIB_DIR/libddscxx.so.0" \
      <<<"$ldd_output" >/dev/null ||
      h2_die DDSCXX_NOT_RESOLVED_TO_PRIVATE_BUNDLE 16
    grep -Eq 'lib(rcl|rmw|ros)' <<<"$ldd_output" &&
      h2_die "UNEXPECTED_ROS_DYNAMIC_DEPENDENCY=$binary" 16
  done
  printf 'PRIVATE_BUNDLE_LDD_OK\n'

  h2_check_no_probe_process
  printf 'H2_COMMON_GATE_PRECONDITIONS_OK\n'
}
