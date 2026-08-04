# Purpose: extract an audited PC2-native Stage 06C-06E bundle from a pinned
# H2 HAL candidate image, then verify the portable archive offline.
# Input: exact image ID, bundle name, output directory, and repository-owned
# PC2 gate scripts under unitreeH2/remote.
# Output: deterministic tar.gz, companion sha256 file, and host gate marker.
# Safety: refuse a different image/architecture/entrypoint/RMW and existing
# output; packaging and verification containers have no network or privileges.
param(
    [string]$Image = "unitree_h2:amd64-live-test-candidate",
    [string]$ExpectedImageId = "sha256:9a7fd813d6cb509efebb8064bae47613a196a64c9490fdee26270f2b3fd12035",
    [string]$BundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260716_r4",
    [string]$OutputDirectory = ""
)

$ErrorActionPreference = "Stop"

# Resolve repository inputs and the output root independently of the caller CWD.
$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$remoteDirectory = Join-Path $unitreeH2Root "remote"
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $unitreeH2Root "runtime_bundle"
}
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path
$remoteDirectory = (Resolve-Path -LiteralPath $remoteDirectory).Path

# Every gate script and its offline schema test must exist before packaging.
$requiredInputs = @(
    "h2_pc2_hal_gate_common.sh",
    "06_pc2_h2_getters_rpc_gate.sh",
    "07_pc2_h2_zero_stop_gate.sh",
    "08_pc2_h2_single_axis_motion_gate.sh",
    "tests/test_h2_gate_schema_offline.sh",
    "README_PC2_H2_HAL_BUNDLE.md"
)
foreach ($name in $requiredInputs) {
    $path = Join-Path $remoteDirectory $name
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing bundle input: $path"
    }
}

# Pin the exact audited image identity and enforce the HAL-only metadata boundary.
$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
if (-not $inspect) {
    throw "Image not found: $Image"
}
if ($inspect.Id -ne $ExpectedImageId) {
    throw "Image ID mismatch: expected $ExpectedImageId, got $($inspect.Id)"
}
if ($inspect.Architecture -ne "amd64") {
    throw "Unexpected image architecture: $($inspect.Architecture)"
}
if ($inspect.Config.Entrypoint -and $inspect.Config.Entrypoint.Count -gt 0) {
    throw "Candidate image must not inherit an entrypoint"
}
if ($inspect.Config.Env | Where-Object { $_ -like "RMW_IMPLEMENTATION=*" }) {
    throw "Candidate HAL image must not set RMW_IMPLEMENTATION"
}
$sdkCommit = $inspect.Config.Labels.'io.robodog.unitree_sdk2.commit'
if (-not $sdkCommit) {
    throw "Missing pinned SDK2 commit label"
}

# Never replace a prior release artifact or its recorded hash.
$archiveName = "$BundleName.tar.gz"
$archivePath = Join-Path $OutputDirectory $archiveName
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path -LiteralPath $archivePath) -or
    (Test-Path -LiteralPath $archiveHashPath)) {
    throw "Refusing to overwrite existing bundle artifact: $archivePath"
}

# Container packaging phase: copy only the runtime allowlist, preserve library
# symlinks, normalize text inputs, record provenance, and create a deterministic
# archive with a full per-file manifest.
$packageScript = @'
set -euo pipefail
root="/work/$BUNDLE_NAME"
mkdir -p "$root/bin" "$root/lib" "$root/config" "$root/scripts/tests" "$root/meta"

# Install the explicitly approved test and contract binaries.
for binary in \
  robot_test_unitree_h2 \
  robot_test_unitree_h2_live_motion \
  unitree_h2_factory_contract_test \
  unitree_h2_direct_api_contract_test \
  unitree_h2_live_motion_plan_test; do
  install -m 0755 "/opt/robodog/bin/$binary" "$root/bin/$binary"
done

# Copy HAL/vendor/SDK runtime libraries with their SONAME symlinks intact.
install -m 0644 /opt/robodog/lib/librobot_hardware.so \
  "$root/lib/librobot_hardware.so"
install -m 0644 /opt/robodog/lib/libmc_sdk_zsl_1_x86_64.so \
  "$root/lib/libmc_sdk_zsl_1_x86_64.so"
cp -a /opt/unitree_robotics/lib/libddsc.so \
  /opt/unitree_robotics/lib/libddsc.so.0 \
  /opt/unitree_robotics/lib/libddscxx.so \
  /opt/unitree_robotics/lib/libddscxx.so.0 \
  "$root/lib/"
cp -a /lib/x86_64-linux-gnu/libyaml-cpp.so \
  /lib/x86_64-linux-gnu/libyaml-cpp.so.0.7 \
  /lib/x86_64-linux-gnu/libyaml-cpp.so.0.7.0 \
  "$root/lib/"
install -m 0644 /opt/robodog/config/unitree_h2.yaml \
  "$root/config/unitree_h2.yaml"

# Copy PC2 gates from the read-only input mount and remove Windows CR endings.
for script in \
  h2_pc2_hal_gate_common.sh \
  06_pc2_h2_getters_rpc_gate.sh \
  07_pc2_h2_zero_stop_gate.sh \
  08_pc2_h2_single_axis_motion_gate.sh; do
  install -m 0755 "/input/$script" "$root/scripts/$script"
  sed -i 's/\r$//' "$root/scripts/$script"
done
install -m 0755 /input/tests/test_h2_gate_schema_offline.sh \
  "$root/scripts/tests/test_h2_gate_schema_offline.sh"
sed -i 's/\r$//' "$root/scripts/tests/test_h2_gate_schema_offline.sh"
install -m 0644 /input/README_PC2_H2_HAL_BUNDLE.md "$root/README.md"
sed -i 's/\r$//' "$root/README.md"

# Record image and SDK provenance plus the intended stage/scope boundary.
printf '%s\n' "$IMAGE_ID" >"$root/meta/image-id.txt"
printf '%s\n' "$SDK_COMMIT" >"$root/meta/sdk2-commit.txt"
{
  printf 'bundle_name=%s\n' "$BUNDLE_NAME"
  printf 'image_id=%s\n' "$IMAGE_ID"
  printf 'sdk2_commit=%s\n' "$SDK_COMMIT"
  printf 'architecture=amd64\n'
  printf 'build_time_utc=%s\n' "$(date -u --iso-8601=seconds)"
  printf 'scope=Stage06C_getter_Stage06D_zero_Stage06E_single_axis_candidate\n'
  printf 'stage06b_state_source_complete=false\n'
  printf 'pc2_docker_required=false\n'
} >"$root/meta/build-info.txt"

# Hash every release file, record symlink targets, and parse-check all shell.
(
  cd "$root"
  find . -type l -printf '%p -> %l\n' | sort >meta/symlinks.txt
  find . -type f ! -path ./meta/manifest.sha256 -print0 |
    sort -z | xargs -0 sha256sum >meta/manifest.sha256
  bash -n scripts/*.sh scripts/tests/*.sh
)

# Fixed order, timestamp, owner, and group make the archive reproducible.
cd /work
tar --sort=name --mtime='UTC 2026-07-16 00:00:00' \
  --owner=0 --group=0 --numeric-owner \
  -czf "/out/$BUNDLE_NAME.tar.gz" "$BUNDLE_NAME"
cd /out
sha256sum "$BUNDLE_NAME.tar.gz" >"$BUNDLE_NAME.tar.gz.sha256"
printf 'H2_PC2_NATIVE_BUNDLE_CREATED=%s\n' "$BUNDLE_NAME.tar.gz"
'@

# Base64 preserves the multiline Bash payload across PowerShell and Docker.
$packageEncoded = [Convert]::ToBase64String(
    [Text.Encoding]::UTF8.GetBytes($packageScript)
)
# Packaging is offline and unprivileged; only /out is writable persistently.
$packageArgs = @(
    "run", "--rm",
    "--network", "none",
    "--cap-drop", "ALL",
    "--security-opt", "no-new-privileges",
    "--tmpfs", "/work:rw,nosuid,nodev,size=128m",
    "-v", ("{0}:/input:ro" -f $remoteDirectory),
    "-v", ("{0}:/out" -f $OutputDirectory),
    "-e", ("BUNDLE_NAME={0}" -f $BundleName),
    "-e", ("IMAGE_ID={0}" -f $inspect.Id),
    "-e", ("SDK_COMMIT={0}" -f $sdkCommit),
    "--entrypoint", "/bin/bash",
    $Image,
    "-lc", ("echo {0} | base64 -d | bash" -f $packageEncoded)
)
# Preserve Docker's native exit code without changing the global error policy.
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
& docker @packageArgs
$packageExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($packageExitCode -ne 0) {
    throw "Native bundle packaging failed with exit code $packageExitCode"
}

# Independent verification phase: unpack the finished artifact into a fresh
# read-only container, verify hashes/symlinks/dependencies/contracts, and prove
# that live paths still require their explicit authorization gates.
$verifyScript = @'
set -euo pipefail
mkdir -p /verify
tar -xzf "/out/$BUNDLE_NAME.tar.gz" -C /verify
release="/verify/$BUNDLE_NAME"
cd "$release"
# Verify all packaged files and provenance before executing any binary.
sha256sum --check --strict meta/manifest.sha256
grep -Fx "$IMAGE_ID" meta/image-id.txt
grep -Fx "$SDK_COMMIT" meta/sdk2-commit.txt
bash -n scripts/*.sh scripts/tests/*.sh
TMPDIR=/verify bash scripts/tests/test_h2_gate_schema_offline.sh

test "$(readlink lib/libddsc.so.0)" = libddsc.so
test "$(readlink lib/libddscxx.so.0)" = libddscxx.so
test "$(readlink lib/libyaml-cpp.so.0.7)" = libyaml-cpp.so.0.7.0
export LD_LIBRARY_PATH="$release/lib"

# All runtime dependencies must resolve from the portable release, with no ROS.
for binary in robot_test_unitree_h2 robot_test_unitree_h2_live_motion; do
  ldd_output="$(ldd "$release/bin/$binary")"
  printf '%s\n' "$ldd_output"
  ! grep -q 'not found' <<<"$ldd_output"
  grep -F "librobot_hardware.so => $release/lib/librobot_hardware.so" \
    <<<"$ldd_output"
  grep -F "libyaml-cpp.so.0.7 => $release/lib/libyaml-cpp.so.0.7" \
    <<<"$ldd_output"
  grep -F "libmc_sdk_zsl_1_x86_64.so => $release/lib/libmc_sdk_zsl_1_x86_64.so" \
    <<<"$ldd_output"
  grep -F "libddsc.so.0 => $release/lib/libddsc.so.0" <<<"$ldd_output"
  grep -F "libddscxx.so.0 => $release/lib/libddscxx.so.0" <<<"$ldd_output"
  ! grep -Eq 'lib(rcl|rmw|ros)' <<<"$ldd_output"
done

# Run offline contracts and print every supported axis plan without DDS access.
"$release/bin/unitree_h2_factory_contract_test"
"$release/bin/unitree_h2_direct_api_contract_test"
"$release/bin/unitree_h2_live_motion_plan_test"
for axis in x-positive x-negative y-positive y-negative yaw-positive yaw-negative; do
  "$release/bin/robot_test_unitree_h2_live_motion" \
    --print-plan --axis "$axis"
done
# Negative tests ensure unsafe/incomplete invocations cannot accidentally pass.
if "$release/bin/robot_test_unitree_h2" --read-only --zero-stop; then
  exit 61
fi
if "$release/bin/robot_test_unitree_h2_live_motion" --axis x-positive; then
  exit 62
fi
set +e
authorization_output="$(
  "$release/bin/robot_test_unitree_h2_live_motion" \
    --config "$release/config/unitree_h2.yaml" \
    --axis x-positive \
    --expected-fsm 601 \
    --live-motion \
    --acknowledge \
      H2_FALL_ARREST_STAND_FOUR_CASTERS_LOCKED_CLEAR_AREA_SECOND_OPERATOR_REMOTE_HIGH_LEVEL_STOP_CONFIRMED \
    --authorization-file \
      /home/unitree/p2_unitreeH2/build/h2_control_gate_state/invalid \
    2>&1
)"
authorization_rc=$?
set -e
printf '%s\n' "$authorization_output"
[[ "$authorization_rc" -eq 65 ]]
grep -F 'Missing one-time Stage 06E authorization token.' \
  <<<"$authorization_output"
printf 'H2_PC2_NATIVE_BUNDLE_OFFLINE_OK\n'
'@

# Encode and execute the verifier in a separate offline, read-only container.
$verifyEncoded = [Convert]::ToBase64String(
    [Text.Encoding]::UTF8.GetBytes($verifyScript)
)
$verifyArgs = @(
    "run", "--rm",
    "--network", "none",
    "--read-only",
    "--cap-drop", "ALL",
    "--security-opt", "no-new-privileges",
    "--tmpfs", "/verify:rw,exec,nosuid,nodev,size=128m",
    "-v", ("{0}:/out:ro" -f $OutputDirectory),
    "-e", ("BUNDLE_NAME={0}" -f $BundleName),
    "-e", ("IMAGE_ID={0}" -f $inspect.Id),
    "-e", ("SDK_COMMIT={0}" -f $sdkCommit),
    "--entrypoint", "/bin/bash",
    $Image,
    "-lc", ("echo {0} | base64 -d | bash" -f $verifyEncoded)
)
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
& docker @verifyArgs
$verifyExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($verifyExitCode -ne 0) {
    throw "Native bundle verification failed with exit code $verifyExitCode"
}

# Recheck the exported artifact on the Windows host before publishing results.
if (-not (Test-Path -LiteralPath $archivePath -PathType Leaf) -or
    -not (Test-Path -LiteralPath $archiveHashPath -PathType Leaf)) {
    throw "Bundle output is incomplete"
}
$actualHash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expectedHash = ((Get-Content -LiteralPath $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($actualHash -ne $expectedHash) {
    throw "Archive hash mismatch: expected $expectedHash, got $actualHash"
}

# Print immutable release identity only after every host/container gate passes.
$archive = Get-Item -LiteralPath $archivePath
Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "SDK2_COMMIT=$sdkCommit"
Write-Host "BUNDLE_PATH=$archivePath"
Write-Host "BUNDLE_SIZE=$($archive.Length)"
Write-Host "BUNDLE_SHA256=$actualHash"
Write-Host "H2_PC2_NATIVE_BUNDLE_HOST_GATE_OK"
