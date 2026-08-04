# Purpose: verify an H2 runtime candidate and invoke its full in-container gate.
# Input: a local image tag.
# Output: image identity and H2_RUNTIME_IMAGE_HOST_GATE_OK.
# Safety: require amd64 and the exact scope label; run offline, read-only, and
# without capabilities. This proves no live IMU or motion behavior.
param(
    [string]$Image = "unitree_h2:amd64-runtime-candidate"
)

$ErrorActionPreference = "Stop"
# Reject HAL-only, X30, or other incorrectly tagged images on the host.
$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
if (-not $inspect) { throw "Image not found: $Image" }
if ($inspect.Architecture -ne "amd64") {
    throw "Unexpected image architecture: $($inspect.Architecture)"
}
if ($inspect.Config.Labels.'io.robodog.h2.runtime.scope' -ne
    'hal-native-hg-state-ros2-imu-candidate') {
    throw "Unexpected H2 runtime scope label"
}

# Mount the repository-owned verifier read-only into the container.
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$insideCheck = (Resolve-Path (Join-Path $scriptDir "verify_h2_runtime_inside.sh")).Path
$insideMount = "${insideCheck}:/verify_h2_runtime_inside.sh:ro"

# Use a noexec tmpfs for ROS logs while retaining a read-only root filesystem.
docker run --rm `
    --network none `
    --read-only `
    --cap-drop ALL `
    --security-opt no-new-privileges `
    --tmpfs /tmp:rw,noexec,nosuid,size=64m `
    --volume $insideMount `
    --entrypoint /bin/bash `
    $Image /verify_h2_runtime_inside.sh
if ($LASTEXITCODE -ne 0) {
    throw "H2 runtime container verification failed: $LASTEXITCODE"
}

# Print recordable image identity only after the gate passes.
Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "IMAGE_ARCH=$($inspect.Architecture)"
Write-Host "H2_RUNTIME_IMAGE_HOST_GATE_OK"
