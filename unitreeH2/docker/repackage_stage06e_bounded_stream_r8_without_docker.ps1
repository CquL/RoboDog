# Purpose: produce r8 with bounded 20 Hz motion streaming and no-motion
# no-promotion gates, using a thin WSL rebuild over the audited r7 carrier.
# Input: exact r7 parent, pinned yaml-cpp package, current HAL/SDK2, PC2 gates.
# Output: deterministic r8 tar.gz, sha256 companion, and host success marker.
# Safety: pin all reusable binaries by parent/package hashes, refuse overwrite,
# run offline plan/contracts, and reverify the final archive before success.
param(
    [string]$ParentBundle = "",
    [string]$YamlDevDeb = "",
    [string]$OutputDirectory = ""
)

$ErrorActionPreference = "Stop"

# Convert an existing drive-qualified Windows path to a WSL mount path.
function ConvertTo-WslPath([string]$Path) {
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '^[A-Za-z]:\\') {
        throw "Expected a drive-qualified Windows path: $resolved"
    }
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $tail = $resolved.Substring(2).Replace('\', '/')
    return "/mnt/$drive$tail"
}

# Resolve fixed release identity and all repository/dependency roots.
$bundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8"
$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $unitreeH2Root
$robotHardwareSource = Join-Path $workspaceRoot "robot_hardware\robot_hardware"
$sdkSource = Join-Path $unitreeH2Root "vendor\unitree_sdk2"
$remoteDirectory = Join-Path $unitreeH2Root "remote"

if (-not $ParentBundle) {
    $ParentBundle = Join-Path $unitreeH2Root `
        "runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7.tar.gz"
}
if (-not $YamlDevDeb) {
    $YamlDevDeb = Join-Path $unitreeH2Root `
        "downloads\libyaml-cpp-dev_0.7.0+dfsg-8build1_amd64.deb"
}
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $unitreeH2Root "runtime_bundle"
}

$ParentBundle = (Resolve-Path -LiteralPath $ParentBundle).Path
$YamlDevDeb = (Resolve-Path -LiteralPath $YamlDevDeb).Path
$robotHardwareSource = (Resolve-Path -LiteralPath $robotHardwareSource).Path
$sdkSource = (Resolve-Path -LiteralPath $sdkSource).Path
$remoteDirectory = (Resolve-Path -LiteralPath $remoteDirectory).Path
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

# Validate every thin-rebuild source and gate script before entering WSL.
$requiredInputs = @(
    (Join-Path $robotHardwareSource "robot_test_unitree_h2_live_motion.cpp"),
    (Join-Path $robotHardwareSource "tests\unitree_h2_live_motion_plan_test.cpp"),
    (Join-Path $robotHardwareSource "include\unitree\unitree_h2_live_motion_plan.h"),
    (Join-Path $sdkSource "include\unitree\robot\h2\loco\h2_loco_client.hpp"),
    (Join-Path $remoteDirectory "h2_pc2_hal_gate_common.sh"),
    (Join-Path $remoteDirectory "08_pc2_h2_single_axis_motion_gate.sh"),
    (Join-Path $remoteDirectory "tests\test_h2_gate_schema_offline.sh"),
    (Join-Path $remoteDirectory "README_PC2_H2_HAL_BUNDLE.md")
)
foreach ($path in $requiredInputs) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing r8 input: $path"
    }
}

# Pin the r7 carrier and offline yaml-cpp development package exactly.
$expectedParentHash = "3612e704a0472ba25a751146824f994e41e2c358bd12b2eaf95aae76e1abbebe"
$actualParentHash = (Get-FileHash -LiteralPath $ParentBundle -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualParentHash -ne $expectedParentHash) {
    throw "r7 parent hash mismatch: expected $expectedParentHash, got $actualParentHash"
}
$expectedYamlDebHash = "28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a"
$actualYamlDebHash = (Get-FileHash -LiteralPath $YamlDevDeb -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualYamlDebHash -ne $expectedYamlDebHash) {
    throw "yaml-cpp development package hash mismatch: expected $expectedYamlDebHash, got $actualYamlDebHash"
}

# Never replace an earlier release or its hash record.
$archivePath = Join-Path $OutputDirectory "$bundleName.tar.gz"
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path -LiteralPath $archivePath) -or
    (Test-Path -LiteralPath $archiveHashPath)) {
    throw "Refusing to overwrite existing r8 artifact: $archivePath"
}

# WSL phase: validate/extract r7, rebuild the bounded-stream HAL/test path,
# apply Stage 06E observation gates, run contracts/negative cases, produce a
# deterministic archive, and verify the extracted release.
$bash = @'
set -Eeuo pipefail

parent_bundle="$1"
src="$2"
sdk="$3"
remote_dir="$4"
yaml_deb="$5"
out_dir="$6"

parent_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r7
bundle_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8
expected_parent_sha256=3612e704a0472ba25a751146824f994e41e2c358bd12b2eaf95aae76e1abbebe
expected_yaml_deb_sha256=28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a
expected_sdk_tree_sha256=c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b
expected_h2_loco_header_sha256=1102d1744af1a5364726dbd6c4f5bbefa681401dcd63972ddb98fda9b9d5f367
sdk_commit=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

archive="$out_dir/$bundle_name.tar.gz"
archive_hash="$archive.sha256"
test ! -e "$archive"
test ! -e "$archive_hash"

for tool in bash sha256sum tar g++ dpkg-deb install sed grep find sort xargs \
  awk diff cmp ldd readelf file date basename mktemp; do
  command -v "$tool" >/dev/null
done
source /etc/os-release
test "$ID" = ubuntu
test "$VERSION_ID" = 22.04
test "$(uname -m)" = x86_64

printf '%s  %s\n' "$expected_parent_sha256" "$parent_bundle" |
  sha256sum --check --strict -
printf '%s  %s\n' "$expected_yaml_deb_sha256" "$yaml_deb" |
  sha256sum --check --strict -

sdk_tree_sha256="$({
  cd "$sdk"
  find include thirdparty/include -type f -print0 |
    sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
})"
test "$sdk_tree_sha256" = "$expected_sdk_tree_sha256"
printf '%s  %s\n' "$expected_h2_loco_header_sha256" \
  "$sdk/include/unitree/robot/h2/loco/h2_loco_client.hpp" |
  sha256sum --check --strict -

work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
mkdir -p "$work/yaml-dev" "$work/parent-copy" "$work/verify"
dpkg-deb -x "$yaml_deb" "$work/yaml-dev"
test -r "$work/yaml-dev/usr/include/yaml-cpp/yaml.h"

tar -xzf "$parent_bundle" -C "$work"
parent="$work/$parent_name"
release="$work/$bundle_name"
test -d "$parent"
mv -- "$parent" "$release"
tar -xzf "$parent_bundle" -C "$work/parent-copy"
parent_copy="$work/parent-copy/$parent_name"

common_compile_flags=(
  -std=c++17 -O2 -Wall -Wextra -Wpedantic
  -I"$src/include"
  -I"$src/include/unitree"
  -I"$src/include/zsibot"
  -I"$src/include/deep_robotics"
  -I"$sdk/include"
  -I"$sdk/thirdparty/include"
  -I"$sdk/thirdparty/include/ddscxx"
  -I"$work/yaml-dev/usr/include"
)

g++ "${common_compile_flags[@]}" \
  "$src/tests/unitree_h2_live_motion_plan_test.cpp" \
  -o "$work/unitree_h2_live_motion_plan_test"

g++ "${common_compile_flags[@]}" \
  "$src/robot_test_unitree_h2_live_motion.cpp" \
  -L"$release/lib" \
  -Wl,-rpath,'$ORIGIN/../lib' \
  -Wl,-rpath-link,"$release/lib" \
  -lrobot_hardware -lyaml-cpp -lmc_sdk_zsl_1_x86_64 \
  -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2_live_motion"

install -m 0755 "$work/unitree_h2_live_motion_plan_test" \
  "$release/bin/unitree_h2_live_motion_plan_test"
install -m 0755 "$work/robot_test_unitree_h2_live_motion" \
  "$release/bin/robot_test_unitree_h2_live_motion"

for relative in \
  h2_pc2_hal_gate_common.sh \
  08_pc2_h2_single_axis_motion_gate.sh \
  tests/test_h2_gate_schema_offline.sh; do
  install -m 0755 "$remote_dir/$relative" "$release/scripts/$relative"
  sed -i 's/\r$//' "$release/scripts/$relative"
done
install -m 0644 "$remote_dir/README_PC2_H2_HAL_BUNDLE.md" \
  "$release/README.md"
sed -i 's/\r$//' "$release/README.md"

{
  printf 'parent_bundle=%s\n' "$(basename "$parent_bundle")"
  printf 'parent_bundle_sha256=%s\n' "$expected_parent_sha256"
  printf 'repackage_reason=stage06e_bounded_single_axis_stream\n'
  printf 'sdk2_commit=%s\n' "$sdk_commit"
  printf 'sdk2_include_tree_sha256=%s\n' "$sdk_tree_sha256"
  printf 'yaml_cpp_dev_package=%s\n' "$(basename "$yaml_deb")"
  printf 'yaml_cpp_dev_package_sha256=%s\n' "$expected_yaml_deb_sha256"
  printf 'live_motion_source_sha256=%s\n' \
    "$(sha256sum "$src/robot_test_unitree_h2_live_motion.cpp" | awk '{print $1}')"
  printf 'live_motion_plan_header_sha256=%s\n' \
    "$(sha256sum "$src/include/unitree/unitree_h2_live_motion_plan.h" | awk '{print $1}')"
  printf 'live_motion_plan_test_sha256=%s\n' \
    "$(sha256sum "$src/tests/unitree_h2_live_motion_plan_test.cpp" | awk '{print $1}')"
  printf 'gate_common_sha256=%s\n' \
    "$(sha256sum "$remote_dir/h2_pc2_hal_gate_common.sh" | awk '{print $1}')"
  printf 'stage06e_gate_script_sha256=%s\n' \
    "$(sha256sum "$remote_dir/08_pc2_h2_single_axis_motion_gate.sh" | awk '{print $1}')"
  printf 'gate_schema_test_sha256=%s\n' \
    "$(sha256sum "$remote_dir/tests/test_h2_gate_schema_offline.sh" | awk '{print $1}')"
  printf 'repackage_time_utc=2026-07-21T00:00:00Z\n'
} >>"$release/meta/build-info.txt"

(
  cd "$release"
  find . -type f ! -path ./meta/manifest.sha256 -print0 |
    sort -z | xargs -0 sha256sum >meta/manifest.sha256
)

for tree in lib config; do
  diff -r --no-dereference "$parent_copy/$tree" "$release/$tree"
done
for relative in \
  bin/robot_test_unitree_h2 \
  bin/unitree_h2_factory_contract_test \
  bin/unitree_h2_direct_api_contract_test \
  scripts/06_pc2_h2_getters_rpc_gate.sh \
  scripts/07_pc2_h2_zero_stop_gate.sh \
  meta/image-id.txt \
  meta/sdk2-commit.txt \
  meta/symlinks.txt; do
  cmp "$parent_copy/$relative" "$release/$relative"
done

verify_release() {
  local target="$1"
  local schema_output plan shortest_plan ldd_output
  (
    cd "$target"
    sha256sum --check --strict meta/manifest.sha256
    bash -n scripts/*.sh scripts/tests/*.sh
  )
  schema_output="$(TMPDIR="$work" bash "$target/scripts/tests/test_h2_gate_schema_offline.sh")"
  printf '%s\n' "$schema_output"
  grep -F 'H2_GATE_SCHEMA_OFFLINE_OK' <<<"$schema_output" >/dev/null

  export LD_LIBRARY_PATH="$target/lib"
  "$target/bin/unitree_h2_factory_contract_test"
  "$target/bin/unitree_h2_direct_api_contract_test"
  "$target/bin/unitree_h2_live_motion_plan_test"

  plan="$("$target/bin/robot_test_unitree_h2_live_motion" \
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

  shortest_plan="$("$target/bin/robot_test_unitree_h2_live_motion" \
    --print-plan --axis x-positive \
    --linear-speed 0.08 --yaw-speed 0.08 --stream-ms 250)"
  grep -F 'stream_ms=250' <<<"$shortest_plan" >/dev/null
  grep -F 'expected_rpc_count=5' <<<"$shortest_plan" >/dev/null

  expect_exit_64() {
    set +e
    "$target/bin/robot_test_unitree_h2_live_motion" "$@" >/dev/null 2>&1
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
  expect_exit_64 --print-plan --axis x-positive \
    --stream-ms 1000 --stream-ms 1000

  grep -F 'H2_LIVE_SINGLE_AXIS_STREAM_RPC_OK axis=' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  grep -F 'max_observed_send_gap_ms' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  grep -F 'H2_STAGE06E_NO_MOTION_OBSERVED axis=' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  grep -F 'stage06e_gate_written=0' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  grep -F 'PHYSICAL_MOTION_NOT_OBSERVED_NO_STAGE06E_GATE 52' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  grep -F 'Type BOUNDED_STREAM_OBSERVED_SAFE exactly' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" >/dev/null
  ! grep -R -F 'TINY_PULSE_OBSERVED_SAFE' \
    "$target/scripts" "$target/README.md"
  local no_motion_line write_gate_line
  no_motion_line="$(grep -n 'H2_STAGE06E_NO_MOTION_OBSERVED axis=' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" | cut -d: -f1)"
  write_gate_line="$(grep -n '^h2_write_gate 06e' \
    "$target/scripts/08_pc2_h2_single_axis_motion_gate.sh" | cut -d: -f1)"
  test -n "$no_motion_line"
  test -n "$write_gate_line"
  test "$no_motion_line" -lt "$write_gate_line"

  ldd_output="$(ldd "$target/bin/robot_test_unitree_h2_live_motion")"
  printf '%s\n' "$ldd_output"
  ! grep -q 'not found' <<<"$ldd_output"
  grep -F "librobot_hardware.so => $target/lib/librobot_hardware.so" \
    <<<"$ldd_output" >/dev/null
  grep -F "libyaml-cpp.so.0.7 => $target/lib/libyaml-cpp.so.0.7" \
    <<<"$ldd_output" >/dev/null
  grep -F "libmc_sdk_zsl_1_x86_64.so => $target/lib/libmc_sdk_zsl_1_x86_64.so" \
    <<<"$ldd_output" >/dev/null
  grep -F "libddsc.so.0 => $target/lib/libddsc.so.0" <<<"$ldd_output" >/dev/null
  grep -F "libddscxx.so.0 => $target/lib/libddscxx.so.0" <<<"$ldd_output" >/dev/null
  ! grep -Eq 'lib(rcl|rmw|ros)' <<<"$ldd_output"
  readelf -d "$target/bin/robot_test_unitree_h2_live_motion" |
    grep -F 'Library runpath: [$ORIGIN/../lib]' >/dev/null
}

verify_release "$release"

tar --sort=name --mtime='UTC 2026-07-21 00:00:00' \
  --owner=0 --group=0 --numeric-owner \
  -czf "$archive" -C "$work" "$bundle_name"
(
  cd "$out_dir"
  sha256sum "$bundle_name.tar.gz" >"$bundle_name.tar.gz.sha256"
)

tar -xzf "$archive" -C "$work/verify"
verified="$work/verify/$bundle_name"
verify_release "$verified"

printf 'R8_PARENT_SHA256=%s\n' "$expected_parent_sha256"
printf 'R8_SDK_INCLUDE_TREE_SHA256=%s\n' "$sdk_tree_sha256"
printf 'R8_BUNDLE_PATH=%s\n' "$archive"
printf 'R8_BUNDLE_SHA256=%s\n' "$(awk '{print $1}' "$archive_hash")"
printf 'R8_BOUNDED_STREAM_REPACKAGE_OK\n'
'@

# Base64 preserves the Bash payload through PowerShell and WSL argument parsing.
$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$args = @(
    "-lc",
    ("echo {0} | base64 -d | bash -s -- '{1}' '{2}' '{3}' '{4}' '{5}' '{6}'" -f `
        $encoded,
        (ConvertTo-WslPath $ParentBundle),
        (ConvertTo-WslPath $robotHardwareSource),
        (ConvertTo-WslPath $sdkSource),
        (ConvertTo-WslPath $remoteDirectory),
        (ConvertTo-WslPath $YamlDevDeb),
        (ConvertTo-WslPath $OutputDirectory))
)
# Treat any WSL build or gate failure as a host packaging failure.
& bash @args
if ($LASTEXITCODE -ne 0) {
    throw "WSL r8 thin rebuild/verification failed with exit code $LASTEXITCODE"
}

# Recompute and compare the final archive hash on the Windows host.
$hash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expected = ((Get-Content -LiteralPath $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($hash -ne $expected) {
    throw "r8 archive hash mismatch: expected $expected, got $hash"
}

Write-Host "R8_BUNDLE_PATH=$archivePath"
Write-Host "R8_BUNDLE_SHA256=$hash"
Write-Host "R8_BOUNDED_STREAM_REPACKAGE_HOST_OK"
