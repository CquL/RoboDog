param(
    [string]$ParentBundle = "",
    [string]$YamlDevDeb = "",
    [string]$OutputDirectory = ""
)

$ErrorActionPreference = "Stop"

function ConvertTo-WslPath([string]$Path) {
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '^[A-Za-z]:\\') {
        throw "Expected a drive-qualified Windows path: $resolved"
    }
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $tail = $resolved.Substring(2).Replace('\', '/')
    return "/mnt/$drive$tail"
}

$parentName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r11"
$bundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r12"
$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $unitreeH2Root
$robotHardwareSource = Join-Path $workspaceRoot "robot_hardware\robot_hardware"
$sdkSource = Join-Path $unitreeH2Root "vendor\unitree_sdk2"
$remoteDirectory = Join-Path $unitreeH2Root "remote"

if (-not $ParentBundle) {
    $ParentBundle = Join-Path $unitreeH2Root `
        "runtime_bundle\$parentName.tar.gz"
}
if (-not $YamlDevDeb) {
    $YamlDevDeb = Join-Path $unitreeH2Root `
        "downloads\libyaml-cpp-dev_0.7.0+dfsg-8build1_amd64.deb"
}
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $unitreeH2Root "runtime_bundle"
}

$requiredInputs = @(
    $ParentBundle,
    $YamlDevDeb,
    (Join-Path $robotHardwareSource "src\deep_robotics\deep_robotics_x30.cpp"),
    (Join-Path $robotHardwareSource "src\unitree\unitree_dog.cpp"),
    (Join-Path $robotHardwareSource "src\unitree\unitree_h2.cpp"),
    (Join-Path $robotHardwareSource "src\zsibot\zsibot_zsl_one.cpp"),
    (Join-Path $robotHardwareSource "src\robot_hardware_constant.cpp"),
    (Join-Path $robotHardwareSource "robot_test_unitree_h2.cpp"),
    (Join-Path $robotHardwareSource "tests\unitree_h2_factory_contract_test.cpp"),
    (Join-Path $robotHardwareSource "tests\unitree_h2_direct_api_contract_test.cpp"),
    (Join-Path $robotHardwareSource "config\unitree_h2.yaml"),
    (Join-Path $robotHardwareSource "config\unitree_h2_live.yaml"),
    (Join-Path $remoteDirectory "07_pc2_h2_zero_stop_gate.sh"),
    (Join-Path $remoteDirectory "09_pc2_h2_velocity_probe.sh"),
    (Join-Path $remoteDirectory "h2_pc2_hal_gate_common.sh"),
    (Join-Path $sdkSource "lib\x86_64\libunitree_sdk2.a")
)
foreach ($path in $requiredInputs) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing r12 input: $path"
    }
}

$ParentBundle = (Resolve-Path -LiteralPath $ParentBundle).Path
$YamlDevDeb = (Resolve-Path -LiteralPath $YamlDevDeb).Path
$robotHardwareSource = (Resolve-Path -LiteralPath $robotHardwareSource).Path
$sdkSource = (Resolve-Path -LiteralPath $sdkSource).Path
$remoteDirectory = (Resolve-Path -LiteralPath $remoteDirectory).Path
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

$expectedParentHash = "c90e169afc280786e445cfbc118ec418f8cf4cf64768c772640c7707860564d7"
$expectedYamlHash = "28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a"
$actualParentHash =
    (Get-FileHash -LiteralPath $ParentBundle -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualParentHash -ne $expectedParentHash) {
    throw "r11 parent bundle hash mismatch: expected $expectedParentHash, got $actualParentHash"
}
$actualYamlHash =
    (Get-FileHash -LiteralPath $YamlDevDeb -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualYamlHash -ne $expectedYamlHash) {
    throw "yaml-cpp development package hash mismatch: expected $expectedYamlHash, got $actualYamlHash"
}

$archivePath = Join-Path $OutputDirectory "$bundleName.tar.gz"
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path -LiteralPath $archivePath) -or
    (Test-Path -LiteralPath $archiveHashPath)) {
    throw "Refusing to overwrite existing r12 artifact: $archivePath"
}

$bash = @'
set -Eeuo pipefail

parent_bundle="$1"
src="$2"
sdk="$3"
remote="$4"
yaml_deb="$5"
out="$6"

parent_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r11
bundle_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r12
expected_parent=c90e169afc280786e445cfbc118ec418f8cf4cf64768c772640c7707860564d7
expected_yaml=28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a
expected_sdk_tree=c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b
sdk_commit=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

for tool in bash sha256sum tar g++ dpkg-deb install sed grep find sort \
  xargs awk ldd readelf file cmp; do
  command -v "$tool" >/dev/null
done
source /etc/os-release
test "$ID" = ubuntu
test "$VERSION_ID" = 22.04
test "$(uname -m)" = x86_64

printf '%s  %s\n' "$expected_parent" "$parent_bundle" |
  sha256sum --check --strict -
printf '%s  %s\n' "$expected_yaml" "$yaml_deb" |
  sha256sum --check --strict -
sdk_tree="$({
  cd "$sdk"
  find include thirdparty/include -type f -print0 |
    sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
})"
test "$sdk_tree" = "$expected_sdk_tree"

work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
mkdir -p "$work/yaml" "$work/verify"
dpkg-deb -x "$yaml_deb" "$work/yaml"
test -r "$work/yaml/usr/include/yaml-cpp/yaml.h"
tar -xzf "$parent_bundle" -C "$work"
test -d "$work/$parent_name"

# Validate every file in the accepted r11 parent before using it as the
# dependency/runtime carrier.
(
  cd "$work/$parent_name"
  sha256sum --check --strict meta/manifest.sha256
)

release="$work/$bundle_name"
mv -- "$work/$parent_name" "$release"

flags=(
  -std=c++17 -O2 -Wall -Wextra -Wpedantic
  -I"$src/include"
  -I"$src/include/unitree"
  -I"$src/include/zsibot"
  -I"$src/include/deep_robotics"
  -I"$sdk/include"
  -I"$sdk/thirdparty/include"
  -I"$sdk/thirdparty/include/ddscxx"
  -I"$work/yaml/usr/include"
)
runtime_link=(
  -L"$release/lib"
  -Wl,-rpath,'$ORIGIN/../lib'
  -Wl,-rpath-link,"$release/lib"
)

# Rebuild the HAL and the one supported H2 runtime test entry from the current
# merged source tree. The r11 HAL and runtime binaries are never reused.
g++ "${flags[@]}" -fPIC -shared \
  "$src/src/deep_robotics/deep_robotics_x30.cpp" \
  "$src/src/unitree/unitree_dog.cpp" \
  "$src/src/unitree/unitree_h2.cpp" \
  "$src/src/zsibot/zsibot_zsl_one.cpp" \
  "$src/src/robot_hardware_constant.cpp" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" \
  -L"$release/lib" -Wl,-rpath,'$ORIGIN' -Wl,-rpath-link,"$release/lib" \
  -Wl,-soname,librobot_hardware.so \
  -lyaml-cpp -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/librobot_hardware.so"

g++ "${flags[@]}" "$src/robot_test_unitree_h2.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2"

g++ "${flags[@]}" "$src/tests/unitree_h2_factory_contract_test.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/unitree_h2_factory_contract_test"

# Fake-SDK seam: this validates the HAL mapping without DDS or a robot.
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I"$src/tests/fakes" -I"$src/include" -I"$src/include/unitree" \
  -I"$work/yaml/usr/include" \
  "$src/tests/unitree_h2_direct_api_contract_test.cpp" \
  "$src/src/unitree/unitree_h2.cpp" \
  "$src/src/robot_hardware_constant.cpp" \
  -L"$release/lib" -Wl,-rpath,'$ORIGIN/../lib' \
  -Wl,-rpath-link,"$release/lib" -lyaml-cpp -pthread \
  -o "$work/unitree_h2_direct_api_contract_test"

install -m 0755 "$work/librobot_hardware.so" \
  "$release/lib/librobot_hardware.so"
for binary in \
  robot_test_unitree_h2 \
  unitree_h2_factory_contract_test \
  unitree_h2_direct_api_contract_test; do
  install -m 0755 "$work/$binary" "$release/bin/$binary"
done

# r12 exposes one H2 runtime test binary. Offline contract binaries remain.
for obsolete in \
  robot_test_unitree_h2_live_motion \
  robot_test_unitree_h2_motion_mode \
  robot_test_unitree_h2_velocity_cli \
  robot_test_unitree_h2_vendor_velocity_cli; do
  rm -f -- "$release/bin/$obsolete"
done
rm -f -- "$release/scripts/08_pc2_h2_single_axis_motion_gate.sh"

install -m 0644 "$src/config/unitree_h2.yaml" \
  "$release/config/unitree_h2.yaml"
install -m 0644 "$src/config/unitree_h2_live.yaml" \
  "$release/config/unitree_h2_live.yaml"
install -m 0755 "$remote/07_pc2_h2_zero_stop_gate.sh" \
  "$release/scripts/07_pc2_h2_zero_stop_gate.sh"
install -m 0755 "$remote/09_pc2_h2_velocity_probe.sh" \
  "$release/scripts/09_pc2_h2_velocity_probe.sh"
install -m 0755 "$remote/h2_pc2_hal_gate_common.sh" \
  "$release/scripts/h2_pc2_hal_gate_common.sh"
sed -i 's/\r$//' \
  "$release/config/unitree_h2.yaml" \
  "$release/config/unitree_h2_live.yaml" \
  "$release/scripts/07_pc2_h2_zero_stop_gate.sh" \
  "$release/scripts/09_pc2_h2_velocity_probe.sh" \
  "$release/scripts/h2_pc2_hal_gate_common.sh"

cat >"$release/README.md" <<'EOF'
# Unitree H2 PC2 r12 unified-test bundle

r12 keeps one H2 runtime test entry point:

    bin/robot_test_unitree_h2

The factory, direct-API, and live-motion-plan executables under `bin/` are
offline contract tests, not additional robot-control entry points.

## Configuration profiles

- `config/unitree_h2.yaml` is the safe/read-only profile. Non-zero motion is
  disabled.
- `config/unitree_h2_live.yaml` is the explicit live-motion profile used by
  the velocity probe. Use it only with the physical test area clear and the
  original remote-stop path ready.

## Current PC2 test scripts

Run the staged getter and protected zero-stop checks as documented by the
existing 06/07 scripts. The current velocity test is:

    bash scripts/09_pc2_h2_velocity_probe.sh 0.50 0 0 1000

`09_pc2_h2_velocity_probe.sh` replaces the former
`08_pc2_h2_single_axis_motion_gate.sh`, which was tied to a deleted legacy
runtime binary and is intentionally absent from r12. Script 09 invokes only
`bin/robot_test_unitree_h2` through RobotFactory/RobotHardwareInterface and
uses `config/unitree_h2_live.yaml`; it does not retain a direct-vendor backend.

Always verify the archive SHA256 and then `meta/manifest.sha256` before use.
An SDK/RPC return code of zero does not by itself prove physical motion; retain
the on-site operator, remote stop, clear-area, and observation requirements.
EOF

cat >>"$release/meta/build-info.txt" <<EOF
parent_bundle=$(basename "$parent_bundle")
parent_bundle_sha256=$expected_parent
release=r12_h2_unified_test
sdk2_commit=$sdk_commit
sdk2_include_tree_sha256=$sdk_tree
robot_hardware_source_tree_sha256=$({ cd "$src"; find include src tests -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'; })
unified_test_source_sha256=$(sha256sum "$src/robot_test_unitree_h2.cpp" | awk '{print $1}')
unitree_h2_config_sha256=$(sha256sum "$src/config/unitree_h2.yaml" | awk '{print $1}')
unitree_h2_live_config_sha256=$(sha256sum "$src/config/unitree_h2_live.yaml" | awk '{print $1}')
stage07_script_sha256=$(sha256sum "$remote/07_pc2_h2_zero_stop_gate.sh" | awk '{print $1}')
stage09_script_sha256=$(sha256sum "$remote/09_pc2_h2_velocity_probe.sh" | awk '{print $1}')
gate_common_sha256=$(sha256sum "$remote/h2_pc2_hal_gate_common.sh" | awk '{print $1}')
runtime_test_entry=bin/robot_test_unitree_h2
repackage_time_utc=2026-07-22T06:00:00Z
EOF

verify_release() {
  local root="$1"
  (
    cd "$root"
    sha256sum --check --strict meta/manifest.sha256
    bash -n scripts/07_pc2_h2_zero_stop_gate.sh
    bash -n scripts/09_pc2_h2_velocity_probe.sh
    bash -n scripts/h2_pc2_hal_gate_common.sh
  )

  test -x "$root/bin/robot_test_unitree_h2"
  test -x "$root/bin/unitree_h2_factory_contract_test"
  test -x "$root/bin/unitree_h2_direct_api_contract_test"
  test -x "$root/bin/unitree_h2_live_motion_plan_test"
  for obsolete in \
    robot_test_unitree_h2_live_motion \
    robot_test_unitree_h2_motion_mode \
    robot_test_unitree_h2_velocity_cli \
    robot_test_unitree_h2_vendor_velocity_cli; do
    test ! -e "$root/bin/$obsolete"
  done
  test "$(find "$root/bin" -maxdepth 1 -type f -name 'robot_test_unitree_h2*' | wc -l)" -eq 1
  test ! -e "$root/scripts/08_pc2_h2_single_axis_motion_gate.sh"

  cmp -s "$root/config/unitree_h2.yaml" "$src/config/unitree_h2.yaml"
  cmp -s "$root/config/unitree_h2_live.yaml" "$src/config/unitree_h2_live.yaml"
  cmp -s "$root/scripts/07_pc2_h2_zero_stop_gate.sh" \
    "$remote/07_pc2_h2_zero_stop_gate.sh"
  cmp -s "$root/scripts/09_pc2_h2_velocity_probe.sh" \
    "$remote/09_pc2_h2_velocity_probe.sh"
  cmp -s "$root/scripts/h2_pc2_hal_gate_common.sh" \
    "$remote/h2_pc2_hal_gate_common.sh"
  grep -F '`09_pc2_h2_velocity_probe.sh` replaces the former' \
    "$root/README.md" >/dev/null
  grep -F '`bin/robot_test_unitree_h2`' "$root/README.md" >/dev/null

  export LD_LIBRARY_PATH="$root/lib"
  "$root/bin/unitree_h2_factory_contract_test" |
    grep -F 'UNITREE_H2_FACTORY_CONTRACT_OK'
  "$root/bin/unitree_h2_direct_api_contract_test" |
    grep -F 'UNITREE_H2_DIRECT_API_CONTRACT_OK'

  for artifact in \
    "$root/lib/librobot_hardware.so" \
    "$root/bin/robot_test_unitree_h2" \
    "$root/bin/unitree_h2_factory_contract_test" \
    "$root/bin/unitree_h2_direct_api_contract_test" \
    "$root/bin/unitree_h2_live_motion_plan_test"; do
    file "$artifact" | grep -F 'ELF 64-bit' >/dev/null
    ldd_output="$(ldd "$artifact")"
    printf '%s\n' "$ldd_output"
    ! grep -q 'not found' <<<"$ldd_output"
  done
  readelf -d "$root/lib/librobot_hardware.so" |
    grep -F 'Library runpath: [$ORIGIN]' >/dev/null
  readelf -d "$root/bin/robot_test_unitree_h2" |
    grep -F 'Library runpath: [$ORIGIN/../lib]' >/dev/null

  # Parser/config gates are offline and stop before DDS initialization.
  set +e
  "$root/bin/robot_test_unitree_h2" \
    --config "$root/config/unitree_h2.yaml" --read-only --zero-stop \
    >"$work/conflict.out" 2>&1
  conflict_rc=$?
  "$root/bin/robot_test_unitree_h2" \
    --config "$root/config/unitree_h2_live.yaml" --velocity \
    --vx 0.50 --vy 0 --omega 0 --duration-ms 1000 \
    >"$work/no-execute.out" 2>&1
  no_execute_rc=$?
  "$root/bin/robot_test_unitree_h2" \
    --config "$root/config/unitree_h2_live.yaml" --velocity \
    --vx 0.51 --vy 0 --omega 0 --duration-ms 1000 --execute \
    >"$work/above-envelope.out" 2>&1
  above_envelope_rc=$?
  "$root/bin/robot_test_unitree_h2" \
    --config /definitely/missing-r12.yaml --velocity \
    --vx 0.50 --vy 0 --omega 0 --duration-ms 1000 --execute \
    >"$work/accepted-parser.out" 2>&1
  accepted_parser_rc=$?
  set -e

  test "$conflict_rc" -eq 64
  grep -F 'Usage:' "$work/conflict.out" >/dev/null
  test "$no_execute_rc" -eq 64
  grep -F 'Usage:' "$work/no-execute.out" >/dev/null
  test "$above_envelope_rc" -eq 65
  grep -F 'H2_TEST_CONFIG_WOULD_CLAMP' "$work/above-envelope.out" >/dev/null
  test "$accepted_parser_rc" -eq 65
  grep -F 'H2_TEST_CONFIG_FAILED' "$work/accepted-parser.out" >/dev/null

  grep -F 'allow_motion_commands: false' \
    "$root/config/unitree_h2.yaml" >/dev/null
  grep -F 'allow_motion_commands: true' \
    "$root/config/unitree_h2_live.yaml" >/dev/null
  grep -F 'bin/robot_test_unitree_h2' \
    "$root/scripts/09_pc2_h2_velocity_probe.sh" >/dev/null
  ! grep -F 'vendor_velocity_cli' \
    "$root/scripts/09_pc2_h2_velocity_probe.sh" >/dev/null
}

rm -f -- "$release/meta/manifest.sha256"
(
  cd "$release"
  find . -type f ! -path ./meta/manifest.sha256 -print0 |
    sort -z | xargs -0 sha256sum >meta/manifest.sha256
)
verify_release "$release"

archive="$out/$bundle_name.tar.gz"
hash_file="$archive.sha256"
test ! -e "$archive"
test ! -e "$hash_file"
tar --sort=name --mtime='UTC 2026-07-22 06:00:00' \
  --owner=0 --group=0 --numeric-owner \
  -czf "$archive" -C "$work" "$bundle_name"
(
  cd "$out"
  sha256sum "$bundle_name.tar.gz" >"$bundle_name.tar.gz.sha256"
)

tar -xzf "$archive" -C "$work/verify"
verify_release "$work/verify/$bundle_name"
printf 'R12_PARENT_SHA256=%s\n' "$expected_parent"
printf 'R12_BUNDLE_PATH=%s\n' "$archive"
printf 'R12_BUNDLE_SHA256=%s\n' "$(awk '{print $1}' "$hash_file")"
printf 'R12_UNIFIED_TEST_REPACKAGE_OK\n'
'@

$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$arguments = @(
    "-d", "Ubuntu-22.04", "--", "bash", "-lc",
    ("echo {0} | base64 -d | bash -s -- '{1}' '{2}' '{3}' '{4}' '{5}' '{6}'" -f `
        $encoded,
        (ConvertTo-WslPath $ParentBundle),
        (ConvertTo-WslPath $robotHardwareSource),
        (ConvertTo-WslPath $sdkSource),
        (ConvertTo-WslPath $remoteDirectory),
        (ConvertTo-WslPath $YamlDevDeb),
        (ConvertTo-WslPath $OutputDirectory))
)
& wsl.exe @arguments
if ($LASTEXITCODE -ne 0) {
    throw "WSL r12 rebuild/verification failed with exit code $LASTEXITCODE"
}

$actual = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expected = ((Get-Content -LiteralPath $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($actual -ne $expected) {
    throw "r12 archive hash mismatch: expected $expected, got $actual"
}
Write-Host "R12_BUNDLE_PATH=$archivePath"
Write-Host "R12_BUNDLE_SHA256=$actual"
Write-Host "R12_UNIFIED_TEST_REPACKAGE_HOST_OK"
