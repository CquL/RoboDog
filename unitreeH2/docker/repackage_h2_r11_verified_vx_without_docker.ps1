# Purpose: create r11 with the current shared HAL and verified-vx test path,
# using the audited r9 bundle only as a dependency/runtime carrier.
# Input: exact r9 parent, pinned yaml-cpp package, current HAL/SDK2, PC2 scripts.
# Output: deterministic r11 tar.gz, sha256 companion, and host success marker.
# Safety: rebuild the HAL instead of reusing the parent's HAL, verify mapping via
# fake SDK without DDS, refuse overwrite, and reverify the complete release.
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

# Resolve release identities and all source/dependency roots.
$parentName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r9"
$bundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r11"
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

# Validate every source, dependency, script, and test input before WSL.
$requiredInputs = @(
    $ParentBundle,
    $YamlDevDeb,
    (Join-Path $robotHardwareSource "src\deep_robotics\deep_robotics_x30.cpp"),
    (Join-Path $robotHardwareSource "src\unitree\unitree_dog.cpp"),
    (Join-Path $robotHardwareSource "src\unitree\unitree_h2.cpp"),
    (Join-Path $robotHardwareSource "src\zsibot\zsibot_zsl_one.cpp"),
    (Join-Path $robotHardwareSource "src\robot_hardware_constant.cpp"),
    (Join-Path $robotHardwareSource "robot_test_unitree_h2_velocity_cli.cpp"),
    (Join-Path $robotHardwareSource "tests\unitree_h2_factory_contract_test.cpp"),
    (Join-Path $robotHardwareSource "tests\unitree_h2_direct_api_contract_test.cpp"),
    (Join-Path $robotHardwareSource "config\unitree_h2.yaml"),
    (Join-Path $remoteDirectory "09_pc2_h2_velocity_probe.sh"),
    (Join-Path $remoteDirectory "README_R11_VERIFIED_VX.md"),
    (Join-Path $sdkSource "lib\x86_64\libunitree_sdk2.a")
)
foreach ($path in $requiredInputs) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing r11 input: $path"
    }
}

$ParentBundle = (Resolve-Path -LiteralPath $ParentBundle).Path
$YamlDevDeb = (Resolve-Path -LiteralPath $YamlDevDeb).Path
$robotHardwareSource = (Resolve-Path -LiteralPath $robotHardwareSource).Path
$sdkSource = (Resolve-Path -LiteralPath $sdkSource).Path
$remoteDirectory = (Resolve-Path -LiteralPath $remoteDirectory).Path
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

# Pin the r9 carrier and yaml-cpp development package exactly.
$expectedParentHash = "8fd8fb3d5245a542cf2d0fe6677282bd284a6541b8962781c45e558426407738"
$expectedYamlHash = "28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a"
$actualParentHash =
    (Get-FileHash -LiteralPath $ParentBundle -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualParentHash -ne $expectedParentHash) {
    throw "r9 parent bundle hash mismatch: expected $expectedParentHash, got $actualParentHash"
}
$actualYamlHash =
    (Get-FileHash -LiteralPath $YamlDevDeb -Algorithm SHA256).Hash.ToLowerInvariant()
if ($actualYamlHash -ne $expectedYamlHash) {
    throw "yaml-cpp development package hash mismatch: expected $expectedYamlHash, got $actualYamlHash"
}

# Release artifacts are immutable; never overwrite archive or hash.
$archivePath = Join-Path $OutputDirectory "$bundleName.tar.gz"
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path -LiteralPath $archivePath) -or
    (Test-Path -LiteralPath $archiveHashPath)) {
    throw "Refusing to overwrite existing r11 artifact: $archivePath"
}

# WSL phase: validate/extract r9, rebuild the shared HAL/current test programs,
# run fake-SDK and offline contracts, update release metadata, create a
# deterministic archive, and verify the result in a fresh extraction.
$bash = @'
set -Eeuo pipefail

parent_bundle="$1"
src="$2"
sdk="$3"
remote="$4"
yaml_deb="$5"
out="$6"

parent_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r9
bundle_name=unitree_h2_pc2_native_amd64_stage06c_to_06e_20260722_r11
expected_parent=8fd8fb3d5245a542cf2d0fe6677282bd284a6541b8962781c45e558426407738
expected_yaml=28bc70ebbca5a5464609cb881c996c34c9e830c0fafcda37ada1c6928f81802a
expected_sdk_tree=c95bb23be6da8952dd9f94e68caa3815d45c73019ca5310ad58efbc9e5b3d59b
sdk_commit=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b

for tool in bash sha256sum tar g++ dpkg-deb install sed grep find sort \
  xargs awk ldd readelf file; do
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

# Rebuild the shared HAL itself from the current source tree.  The r9 bundle is
# only the audited dependency/runtime carrier; its old HAL is not reused.
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

g++ "${flags[@]}" "$src/robot_test_unitree_h2_velocity_cli.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/robot_test_unitree_h2_velocity_cli"

g++ "${flags[@]}" "$src/tests/unitree_h2_factory_contract_test.cpp" \
  "${runtime_link[@]}" -lrobot_hardware -lyaml-cpp \
  -lmc_sdk_zsl_1_x86_64 -lddscxx -lddsc -pthread \
  -o "$work/unitree_h2_factory_contract_test"

# Fake-SDK seam: verifies the HAL mapping without loading DDS or contacting H2.
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
  robot_test_unitree_h2_velocity_cli \
  unitree_h2_factory_contract_test \
  unitree_h2_direct_api_contract_test; do
  install -m 0755 "$work/$binary" "$release/bin/$binary"
done
install -m 0644 "$src/config/unitree_h2.yaml" \
  "$release/config/unitree_h2.yaml"
install -m 0755 "$remote/09_pc2_h2_velocity_probe.sh" \
  "$release/scripts/09_pc2_h2_velocity_probe.sh"
install -m 0644 "$remote/README_R11_VERIFIED_VX.md" \
  "$release/README_R11.md"
sed -i 's/\r$//' \
  "$release/config/unitree_h2.yaml" \
  "$release/scripts/09_pc2_h2_velocity_probe.sh" \
  "$release/README_R11.md"

cat >>"$release/meta/build-info.txt" <<EOF
parent_bundle=$(basename "$parent_bundle")
parent_bundle_sha256=$expected_parent
release=r11_h2_verified_vx
sdk2_commit=$sdk_commit
sdk2_include_tree_sha256=$sdk_tree
robot_hardware_source_tree_sha256=$({ cd "$src"; find include src tests -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'; })
unitree_h2_config_sha256=$(sha256sum "$src/config/unitree_h2.yaml" | awk '{print $1}')
velocity_cli_source_sha256=$(sha256sum "$src/robot_test_unitree_h2_velocity_cli.cpp" | awk '{print $1}')
velocity_probe_script_sha256=$(sha256sum "$remote/09_pc2_h2_velocity_probe.sh" | awk '{print $1}')
readme_r11_sha256=$(sha256sum "$remote/README_R11_VERIFIED_VX.md" | awk '{print $1}')
repackage_time_utc=2026-07-22T00:00:00Z
EOF

verify_release() {
  local root="$1"
  (
    cd "$root"
    sha256sum --check --strict meta/manifest.sha256
    bash -n scripts/09_pc2_h2_velocity_probe.sh
  )

  export LD_LIBRARY_PATH="$root/lib"
  "$root/bin/unitree_h2_factory_contract_test" |
    grep -F 'UNITREE_H2_FACTORY_CONTRACT_OK'
  "$root/bin/unitree_h2_direct_api_contract_test" |
    grep -F 'UNITREE_H2_DIRECT_API_CONTRACT_OK'

  for artifact in \
    "$root/lib/librobot_hardware.so" \
    "$root/bin/robot_test_unitree_h2_velocity_cli" \
    "$root/bin/unitree_h2_factory_contract_test" \
    "$root/bin/unitree_h2_direct_api_contract_test"; do
    ldd_output="$(ldd "$artifact")"
    printf '%s\n' "$ldd_output"
    ! grep -q 'not found' <<<"$ldd_output"
  done
  readelf -d "$root/lib/librobot_hardware.so" |
    grep -F 'Library runpath: [$ORIGIN]' >/dev/null
  readelf -d "$root/bin/robot_test_unitree_h2_velocity_cli" |
    grep -F 'Library runpath: [$ORIGIN/../lib]' >/dev/null

  # Parser boundary checks remain offline: 0.51 must be rejected with usage;
  # 0.50 must pass parsing and then fail only on the deliberately missing YAML.
  set +e
  "$root/bin/robot_test_unitree_h2_velocity_cli" \
    --config /definitely/missing-r11.yaml --vx 0.51 --vy 0 --omega 0 \
    --duration-ms 1000 --execute >"$work/reject.out" 2>&1
  reject_rc=$?
  "$root/bin/robot_test_unitree_h2_velocity_cli" \
    --config /definitely/missing-r11.yaml --vx 0.50 --vy 0 --omega 0 \
    --duration-ms 1000 --execute >"$work/accept.out" 2>&1
  accept_rc=$?
  set -e
  test "$reject_rc" -eq 64
  grep -F 'Usage:' "$work/reject.out" >/dev/null
  test "$accept_rc" -eq 65
  grep -F 'H2_HAL_CONFIG_FAILED' "$work/accept.out" >/dev/null

  grep -F 'max_vx: 0.20' "$root/config/unitree_h2.yaml" >/dev/null
  grep -F 'vendor 0.50 0 0 1000' \
    "$root/scripts/09_pc2_h2_velocity_probe.sh" >/dev/null
  grep -F 'hal    0.50 0 0 1000' "$root/README_R11.md" >/dev/null
  ! grep -F 'H2_MOTION_SWITCHER_CHECK' \
    "$root/scripts/09_pc2_h2_velocity_probe.sh" >/dev/null
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
printf 'R11_PARENT_SHA256=%s\n' "$expected_parent"
printf 'R11_BUNDLE_PATH=%s\n' "$archive"
printf 'R11_BUNDLE_SHA256=%s\n' "$(awk '{print $1}' "$hash_file")"
printf 'R11_VERIFIED_VX_REPACKAGE_OK\n'
'@

# Base64 preserves the Bash payload through PowerShell-to-WSL parsing.
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
# Any WSL rebuild or verification failure prevents the r11 success marker.
& wsl.exe @arguments
if ($LASTEXITCODE -ne 0) {
    throw "WSL r11 rebuild/verification failed with exit code $LASTEXITCODE"
}

# Recompute and compare the final archive hash on Windows.
$actual = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expected = ((Get-Content -LiteralPath $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($actual -ne $expected) {
    throw "r11 archive hash mismatch: expected $expected, got $actual"
}
Write-Host "R11_BUNDLE_PATH=$archivePath"
Write-Host "R11_BUNDLE_SHA256=$actual"
Write-Host "R11_VERIFIED_VX_REPACKAGE_HOST_OK"
