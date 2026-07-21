param(
    [string]$Bundle = "",
    [string]$ExpectedSha256 = "3612e704a0472ba25a751146824f994e41e2c358bd12b2eaf95aae76e1abbebe"
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
        "runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz"
}
$Bundle = (Resolve-Path -LiteralPath $Bundle).Path
$actualHash = (Get-FileHash -LiteralPath $Bundle -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualHash -ne $ExpectedSha256.ToLowerInvariant()) {
    throw "r7 bundle hash mismatch: expected $ExpectedSha256, got $actualHash"
}

$bash = @'
set -Eeuo pipefail
bundle="$1"
expected_sha256="$2"
release_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7

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
  TMPDIR="$work" bash scripts/tests/test_h2_gate_schema_offline.sh
)

export LD_LIBRARY_PATH="$release/lib"
"$release/bin/unitree_h2_factory_contract_test"
"$release/bin/unitree_h2_direct_api_contract_test"
"$release/bin/unitree_h2_live_motion_plan_test"

plan="$($release/bin/robot_test_unitree_h2_live_motion \
  --print-plan --axis x-positive \
  --linear-speed 0.08 --yaw-speed 0.08 --pulse-ms 200)"
printf '%s\n' "$plan"
grep -F 'vx=0.080' <<<"$plan" >/dev/null
grep -F 'vendor_duration_s=0.300' <<<"$plan" >/dev/null
grep -F 'local_pulse_ms=200' <<<"$plan" >/dev/null
grep -F 'watchdog_ms=260' <<<"$plan" >/dev/null

set +e
"$release/bin/robot_test_unitree_h2_live_motion" \
  --print-plan --axis x-positive --linear-speed 0.101 >/dev/null 2>&1
reject_rc=$?
set -e
test "$reject_rc" -eq 64

ldd_output="$(ldd "$release/bin/robot_test_unitree_h2_live_motion")"
! grep -q 'not found' <<<"$ldd_output"
grep -F "librobot_hardware.so => $release/lib/librobot_hardware.so" \
  <<<"$ldd_output" >/dev/null
grep -F "libyaml-cpp.so.0.7 => $release/lib/libyaml-cpp.so.0.7" \
  <<<"$ldd_output" >/dev/null
grep -F "libddsc.so.0 => $release/lib/libddsc.so.0" <<<"$ldd_output" >/dev/null
grep -F "libddscxx.so.0 => $release/lib/libddscxx.so.0" <<<"$ldd_output" >/dev/null
! grep -Eq 'lib(rcl|rmw|ros)' <<<"$ldd_output"
readelf -d "$release/bin/robot_test_unitree_h2_live_motion" |
  grep -F 'Library runpath: [$ORIGIN/../lib]' >/dev/null

printf 'R7_OFFLINE_VERIFY_OK\n'
'@

$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$args = @(
    "-lc",
    ("echo {0} | base64 -d | bash -s -- '{1}' '{2}'" -f `
        $encoded,
        (ConvertTo-WslPath $Bundle),
        $ExpectedSha256.ToLowerInvariant())
)
& bash @args
if ($LASTEXITCODE -ne 0) {
    throw "r7 offline verification failed with exit code $LASTEXITCODE"
}

Write-Host "R7_BUNDLE_SHA256=$actualHash"
Write-Host "R7_OFFLINE_VERIFY_HOST_OK"
