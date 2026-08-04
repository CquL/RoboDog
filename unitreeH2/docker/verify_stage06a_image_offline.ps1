# Purpose: verify H2 HAL-only image architecture, entrypoint, dependencies, and
# offline contracts from the host.
# Input: image tag and optional exact ExpectedImageId.
# Output: image identity and H2_STAGE06A_HOST_GATE_OK.
# Safety: run offline, read-only, and without capabilities; reject ROS/RMW
# coupling and inherited entrypoints. No live robot is contacted.
param(
    [string]$Image = "unitree_h2:amd64-offline",
    [string]$ExpectedImageId = ""
)

$ErrorActionPreference = "Stop"

# Verify image identity, amd64 architecture, and HAL-only metadata first.
$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
if (-not $inspect) {
    throw "Image not found: $Image"
}
if ($inspect.Architecture -ne "amd64") {
    throw "Unexpected image architecture: $($inspect.Architecture)"
}
if ($ExpectedImageId -and $inspect.Id -ne $ExpectedImageId) {
    throw "Image ID mismatch: expected $ExpectedImageId, got $($inspect.Id)"
}
if ($inspect.Config.Entrypoint -and $inspect.Config.Entrypoint.Count -gt 0) {
    throw "HAL-only image must not inherit an entrypoint: $($inspect.Config.Entrypoint -join ' ')"
}
if ($inspect.Config.Env | Where-Object { $_ -like "RMW_IMPLEMENTATION=*" }) {
    throw "HAL-only image must not select a ROS 2 RMW implementation"
}

# Inside the container, verify artifacts, dependencies, and both contracts.
$containerCheck = @'
set -euo pipefail
test -x /opt/robodog/bin/unitree_h2_factory_contract_test
test -x /opt/robodog/bin/unitree_h2_direct_api_contract_test
if env | grep -q '^RMW_IMPLEMENTATION='; then
  echo 'Unexpected RMW_IMPLEMENTATION in container' >&2
  exit 20
fi
if ldd /opt/robodog/lib/librobot_hardware.so | grep -q 'not found'; then
  echo 'Missing robot_hardware dependency' >&2
  exit 21
fi
if ldd /opt/robodog/lib/librobot_hardware.so | grep -Eq 'lib(rcl|rmw|ros)'; then
  echo 'Unexpected ROS runtime dependency in robot_hardware' >&2
  exit 22
fi
/opt/robodog/bin/unitree_h2_factory_contract_test
/opt/robodog/bin/unitree_h2_direct_api_contract_test
sha256sum \
  /opt/robodog/lib/librobot_hardware.so \
  /opt/robodog/bin/unitree_h2_factory_contract_test \
  /opt/robodog/bin/unitree_h2_direct_api_contract_test \
  /opt/unitree_robotics/lib/libunitree_sdk2.a \
  /opt/unitree_robotics/lib/libddsc.so \
  /opt/unitree_robotics/lib/libddscxx.so
echo H2_STAGE06A_IMAGE_OFFLINE_OK
'@

# Force offline/read-only/no-capability execution and override the entrypoint.
docker run --rm `
    --network none `
    --read-only `
    --cap-drop ALL `
    --security-opt no-new-privileges `
    --entrypoint /bin/bash `
    $Image -lc $containerCheck

if ($LASTEXITCODE -ne 0) {
    throw "Stage 06A container verification failed with exit code $LASTEXITCODE"
}

# Success proves the offline image contract, not PC2 live control.
Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "IMAGE_ARCH=$($inspect.Architecture)"
Write-Host "H2_STAGE06A_HOST_GATE_OK"
