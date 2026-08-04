# Purpose: create the r9 diagnostic bundle by rebuilding H2 diagnostic/runtime
# binaries in WSL over the audited r8 dependency carrier.
# Input: exact r8 parent, pinned yaml-cpp package, current HAL/SDK2, PC2 scripts.
# Output: deterministic r9 tar.gz, sha256 companion, and host success marker.
# Safety: parent/package hashes are mandatory, existing output is protected,
# offline contracts run before packaging, and the final archive is reverified.
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

# Resolve release identity and all local source/dependency roots.
$bundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r9"
$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $unitreeH2Root
$robotHardwareSource = Join-Path $workspaceRoot "robot_hardware\robot_hardware"
$sdkSource = Join-Path $unitreeH2Root "vendor\unitree_sdk2"
$remoteDirectory = Join-Path $unitreeH2Root "remote"

if (-not $ParentBundle) {
    $ParentBundle = Join-Path $unitreeH2Root `
        "runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8.tar.gz"
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

# Pin the reusable r8 carrier and offline yaml-cpp package.
$expectedParentHash = "ac51bd6544eaea8467cf9472aea74bc29dba1889671c50a20b26d1976284e0cd"
$expectedYamlHash = "28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a"
if ((Get-FileHash $ParentBundle -Algorithm SHA256).Hash.ToLowerInvariant() -ne
    $expectedParentHash) {
    throw "r8 parent bundle hash mismatch"
}
if ((Get-FileHash $YamlDevDeb -Algorithm SHA256).Hash.ToLowerInvariant() -ne
    $expectedYamlHash) {
    throw "yaml-cpp development package hash mismatch"
}

# Refuse to overwrite any prior r9 release artifact.
$archivePath = Join-Path $OutputDirectory "$bundleName.tar.gz"
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path $archivePath) -or (Test-Path $archiveHashPath)) {
    throw "Refusing to overwrite existing r9 artifact: $archivePath"
}

# WSL phase: verify/extract r8, rebuild diagnostic binaries from current source,
# update approved scripts/metadata, run offline checks, create a reproducible
# archive, and verify the extracted r9 release.
$bash = @'
set -Eeuo pipefail

parent_bundle="$1"
src="$2"
sdk="$3"
remote="$4"
yaml_deb="$5"
out="$6"

parent_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260721_r8
bundle_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r9
expected_parent=ac51bd6544eaea8467cf9472aea74bc29dba1889671c50a20b26d1976284e0cd
expected_yaml=28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a
expected_sdk_tree=c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b
sdk_commit=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

printf '%s  %s\n' "$expected_parent" "$parent_bundle" | sha256sum --check --strict -
printf '%s  %s\n' "$expected_yaml" "$yaml_deb" | sha256sum --check --strict -
sdk_tree="$({ cd "$sdk"; find include thirdparty/include -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'; })"
test "$sdk_tree" = "$expected_sdk_tree"

work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
mkdir -p "$work/yaml" "$work/verify"
dpkg-deb -x "$yaml_deb" "$work/yaml"
tar -xzf "$parent_bundle" -C "$work"
release="$work/$bundle_name"
mv "$work/$parent_name" "$release"

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

g++ "${flags[@]}" "$src/tests/unitree_h2_live_motion_plan_test.cpp" \
  -o "$work/unitree_h2_live_motion_plan_test"
g++ "${flags[@]}" "$src/robot_test_unitree_h2_live_motion.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2_live_motion"
g++ "${flags[@]}" "$src/robot_test_unitree_h2_motion_mode.cpp" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" "${runtime_link[@]}" \
  -lddscxx -lddsc -pthread -o "$work/robot_test_unitree_h2_motion_mode"
g++ "${flags[@]}" "$src/robot_test_unitree_h2_velocity_cli.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2_velocity_cli"
g++ "${flags[@]}" "$src/robot_test_unitree_h2_vendor_velocity_cli.cpp" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" "${runtime_link[@]}" \
  -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2_vendor_velocity_cli"

for binary in \
  unitree_h2_live_motion_plan_test \
  robot_test_unitree_h2_live_motion \
  robot_test_unitree_h2_motion_mode \
  robot_test_unitree_h2_velocity_cli \
  robot_test_unitree_h2_vendor_velocity_cli; do
  install -m 0755 "$work/$binary" "$release/bin/$binary"
done

for relative in \
  08_pc2_h2_single_axis_motion_gate.sh \
  09_pc2_h2_velocity_probe.sh \
  tests/test_h2_gate_schema_offline.sh; do
  install -m 0755 "$remote/$relative" "$release/scripts/$relative"
  sed -i 's/\r$//' "$release/scripts/$relative"
done
install -m 0755 "$remote/h2_dog_odom_capture.py" \
  "$release/scripts/h2_dog_odom_capture.py"
install -m 0644 "$remote/README_R9.md" "$release/README_R9.md"
install -m 0644 "$remote/README_R9_SIMPLE_VELOCITY.md" \
  "$release/README_R9_SIMPLE_VELOCITY.md"
sed -i 's/\r$//' "$release/scripts/h2_dog_odom_capture.py" \
  "$release/README_R9.md" "$release/README_R9_SIMPLE_VELOCITY.md"

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  "$release/scripts/h2_dog_odom_capture.py"
rm -rf "$release/scripts/__pycache__"
bash -n "$release/scripts/08_pc2_h2_single_axis_motion_gate.sh"
bash -n "$release/scripts/09_pc2_h2_velocity_probe.sh"
bash -n "$release/scripts/tests/test_h2_gate_schema_offline.sh"

{
  printf 'parent_bundle=%s\n' "$(basename "$parent_bundle")"
  printf 'parent_bundle_sha256=%s\n' "$expected_parent"
  printf 'release=r9_h2_motion_diagnostic\n'
  printf 'sdk2_commit=%s\n' "$sdk_commit"
  printf 'sdk2_include_tree_sha256=%s\n' "$sdk_tree"
  printf 'simple_hal_source_sha256=%s\n' "$(sha256sum "$src/robot_test_unitree_h2_velocity_cli.cpp" | awk '{print $1}')"
  printf 'simple_vendor_source_sha256=%s\n' "$(sha256sum "$src/robot_test_unitree_h2_vendor_velocity_cli.cpp" | awk '{print $1}')"
  printf 'simple_probe_script_sha256=%s\n' "$(sha256sum "$remote/09_pc2_h2_velocity_probe.sh" | awk '{print $1}')"
  printf 'repackage_time_utc=2026-07-22T00:00:00Z\n'
} >>"$release/meta/build-info.txt"

verify_release() {
  local root="$1"
  (
    cd "$root"
    sha256sum --check --strict meta/manifest.sha256
    bash -n scripts/*.sh scripts/tests/*.sh
  )
  export LD_LIBRARY_PATH="$root/lib"
  "$root/bin/unitree_h2_factory_contract_test"
  "$root/bin/unitree_h2_direct_api_contract_test"
  "$root/bin/unitree_h2_live_motion_plan_test"
  "$root/bin/robot_test_unitree_h2_live_motion" \
    --print-plan --axis x-positive --linear-speed 0.10 --stream-ms 1000 |
    grep -F 'H2_LIVE_PRINT_PLAN_ONLY_NO_DDS'
  set +e
  "$root/bin/robot_test_unitree_h2_velocity_cli" >/dev/null 2>&1
  local hal_rc=$?
  "$root/bin/robot_test_unitree_h2_vendor_velocity_cli" >/dev/null 2>&1
  local vendor_rc=$?
  set -e
  test "$hal_rc" -eq 64
  test "$vendor_rc" -eq 64
  grep -F 'MotionSwitcher form=0 name=ai' \
    "$root/README_R9_SIMPLE_VELOCITY.md" >/dev/null
  for binary in \
    robot_test_unitree_h2_live_motion \
    robot_test_unitree_h2_motion_mode \
    robot_test_unitree_h2_velocity_cli \
    robot_test_unitree_h2_vendor_velocity_cli; do
    ! ldd "$root/bin/$binary" | grep -q 'not found'
  done
}

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
tar --sort=name --mtime='UTC 2026-07-22 00:00:00' \
  --owner=0 --group=0 --numeric-owner \
  -czf "$archive" -C "$work" "$bundle_name"
(
  cd "$out"
  sha256sum "$bundle_name.tar.gz" >"$bundle_name.tar.gz.sha256"
)

tar -xzf "$archive" -C "$work/verify"
verify_release "$work/verify/$bundle_name"
printf 'R9_BUNDLE_PATH=%s\n' "$archive"
printf 'R9_BUNDLE_SHA256=%s\n' "$(awk '{print $1}' "$hash_file")"
printf 'R9_H2_DIAGNOSTIC_REPACKAGE_OK\n'
'@

# Base64 keeps the multiline Bash payload unchanged across process boundaries.
$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$args = @(
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
# Any WSL build/package/verification error aborts release publication.
& wsl.exe @args
if ($LASTEXITCODE -ne 0) {
    throw "WSL r9 rebuild/verification failed with exit code $LASTEXITCODE"
}

# Verify the final archive against its companion hash from the Windows host.
$actual = (Get-FileHash $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expected = ((Get-Content $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($actual -ne $expected) {
    throw "r9 archive hash mismatch: expected $expected, got $actual"
}
Write-Host "R9_BUNDLE_PATH=$archivePath"
Write-Host "R9_BUNDLE_SHA256=$actual"
Write-Host "R9_H2_DIAGNOSTIC_REPACKAGE_HOST_OK"
