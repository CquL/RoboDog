param(
    [string]$Bundle = "",
    [string]$ExpectedSha256 = "ac51bd6544eaea8467cf9472aea74bc29dba1889671c50a20b26d1976284e0cd"
)

$ErrorActionPreference = "Stop"

function ConvertTo-WslPath([string]$Path) {
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '^[A-Za-z]:\\') {
        throw "Expected a drive-qualified Windows path: $resolved"
    }
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    return "/mnt/$drive$($resolved.Substring(2).Replace('\', '/'))"
}

$unitreeH2Root = Split-Path -Parent $PSScriptRoot
if (-not $Bundle) {
    $Bundle = Join-Path $unitreeH2Root `
        "runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8.tar.gz"
}
$Bundle = (Resolve-Path -LiteralPath $Bundle).Path
$expectedLower = $ExpectedSha256.ToLowerInvariant()
$actualHash = (Get-FileHash -LiteralPath $Bundle -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualHash -ne $expectedLower) {
    throw "r8 bundle hash mismatch: expected $expectedLower, got $actualHash"
}

$bash = @'
set -Eeuo pipefail
bundle="$1"
expected_sha256="$2"
release_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8

printf '%s  %s\n' "$expected_sha256" "$bundle" | sha256sum --check --strict -
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
tar -xzf "$bundle" -C "$work"
release="$work/$release_name"
test -d "$release"

(
  cd "$release"
  sha256sum --check --strict meta/manifest.sha256
  bash -n scripts/*.sh scripts/tests/*.sh
)
schema_output="$(TMPDIR="$work" bash "$release/scripts/tests/test_h2_gate_schema_offline.sh")"
printf '%s\n' "$schema_output"
grep -F 'H2_GATE_SCHEMA_OFFLINE_OK' <<<"$schema_output" >/dev/null

export LD_LIBRARY_PATH="$release/lib"
"$release/bin/unitree_h2_factory_contract_test"
"$release/bin/unitree_h2_direct_api_contract_test"
"$release/bin/unitree_h2_live_motion_plan_test"

plan="$("$release/bin/robot_test_unitree_h2_live_motion" \
  --print-plan --axis x-positive)"
printf '%s\n' "$plan"
for field in \
  'H2_LIVE_PLAN axis=x-positive' \
  'vx=0.080' \
  'linear_speed=0.080' \
  'yaw_speed=0.080' \
  'vendor_duration_s=0.300' \
  'stream_ms=1000' \
  'command_hz=20' \
  'command_period_ms=50' \
  'max_send_gap_ms=100' \
  'expected_rpc_count=20' \
  'watchdog_ms=150' \
  'expected_fsm=601' \
  'H2_LIVE_PRINT_PLAN_ONLY_NO_DDS'; do
  grep -F "$field" <<<"$plan" >/dev/null
done

shortest_plan="$("$release/bin/robot_test_unitree_h2_live_motion" \
  --print-plan --axis x-positive \
  --linear-speed 0.08 --yaw-speed 0.08 --stream-ms 250)"
grep -F 'stream_ms=250' <<<"$shortest_plan" >/dev/null
grep -F 'expected_rpc_count=5' <<<"$shortest_plan" >/dev/null

expect_exit_64() {
  set +e
  "$release/bin/robot_test_unitree_h2_live_motion" "$@" >/dev/null 2>&1
  local rc=$?
  set -e
  test "$rc" -eq 64
}
expect_exit_64 --print-plan --axis x-positive --linear-speed 0.101
expect_exit_64 --print-plan --axis x-positive --linear-speed 0.0805
expect_exit_64 --print-plan --axis x-positive --stream-ms 249
expect_exit_64 --print-plan --axis x-positive --stream-ms 1050
expect_exit_64 --print-plan --axis x-positive --stream-ms 275
expect_exit_64 --print-plan --axis x-positive --pulse-ms 200
expect_exit_64 --print-plan --axis x-positive --stream-ms 1000 --stream-ms 1000

motion_gate="$release/scripts/08_pc2_h2_single_axis_motion_gate.sh"
grep -F 'H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK axis=$axis' "$motion_gate" >/dev/null
grep -F 'max_observed_send_gap_ms' "$motion_gate" >/dev/null
grep -F 'H2_STAGE06E_NO_MOTION_OBSERVED axis=' "$motion_gate" >/dev/null
grep -F 'stage06e_gate_written=0' "$motion_gate" >/dev/null
grep -F 'h2_die PHYSICAL_MOTION_NOT_OBSERVED_NO_STAGE06E_GATE 52' \
  "$motion_gate" >/dev/null
grep -F 'Type BOUNDED_STREAM_OBSERVED_SAFE exactly' "$motion_gate" >/dev/null
! grep -R -F 'TINY_PULSE_OBSERVED_SAFE' "$release/scripts" "$release/README.md"
no_motion_line="$(grep -n '^if h2_observation_reports_no_motion ' \
  "$motion_gate" | cut -d: -f1)"
safety_line="$(grep -n 'Type BOUNDED_STREAM_OBSERVED_SAFE exactly' \
  "$motion_gate" | cut -d: -f1)"
write_gate_line="$(grep -n '^h2_write_gate 06e ' "$motion_gate" | cut -d: -f1)"
test -n "$no_motion_line"
test -n "$safety_line"
test -n "$write_gate_line"
test "$no_motion_line" -lt "$safety_line"
test "$safety_line" -lt "$write_gate_line"

ldd_output="$(ldd "$release/bin/robot_test_unitree_h2_live_motion")"
printf '%s\n' "$ldd_output"
! grep -q 'not found' <<<"$ldd_output"
grep -F "librobot_hardware.so => $release/lib/librobot_hardware.so" \
  <<<"$ldd_output" >/dev/null
grep -F "libyaml-cpp.so.0.7 => $release/lib/libyaml-cpp.so.0.7" \
  <<<"$ldd_output" >/dev/null
grep -F "libmc_sdk_zsl_1_x86_64.so => $release/lib/libmc_sdk_zsl_1_x86_64.so" \
  <<<"$ldd_output" >/dev/null
grep -F "libddsc.so.0 => $release/lib/libddsc.so.0" <<<"$ldd_output" >/dev/null
grep -F "libddscxx.so.0 => $release/lib/libddscxx.so.0" <<<"$ldd_output" >/dev/null
! grep -Eq 'lib(rcl|rmw|ros)' <<<"$ldd_output"
readelf -d "$release/bin/robot_test_unitree_h2_live_motion" |
  grep -F 'Library runpath: [$ORIGIN/../lib]' >/dev/null

printf 'R8_NO_MOTION_NO_PROMOTION_OFFLINE_OK\n'
printf 'R8_OFFLINE_VERIFY_OK\n'
'@

$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$args = @(
    "-lc",
    ("echo {0} | base64 -d | bash -s -- '{1}' '{2}'" -f `
        $encoded,
        (ConvertTo-WslPath $Bundle),
        $expectedLower)
)
& bash @args
if ($LASTEXITCODE -ne 0) {
    throw "r8 offline verification failed with exit code $LASTEXITCODE"
}

Write-Host "R8_BUNDLE_SHA256=$actualHash"
Write-Host "R8_OFFLINE_VERIFY_HOST_OK"
