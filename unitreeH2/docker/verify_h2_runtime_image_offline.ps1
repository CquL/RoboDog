param(
    [string]$Image = "unitree_h2:amd64-runtime-candidate"
)

$ErrorActionPreference = "Stop"
$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
if (-not $inspect) { throw "Image not found: $Image" }
if ($inspect.Architecture -ne "amd64") {
    throw "Unexpected image architecture: $($inspect.Architecture)"
}
if ($inspect.Config.Labels.'io.robodog.h2.runtime.scope' -ne
    'hal-native-hg-state-ros2-imu-candidate') {
    throw "Unexpected H2 runtime scope label"
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$insideCheck = (Resolve-Path (Join-Path $scriptDir "verify_h2_runtime_inside.sh")).Path
$insideMount = "${insideCheck}:/verify_h2_runtime_inside.sh:ro"

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

Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "IMAGE_ARCH=$($inspect.Architecture)"
Write-Host "H2_RUNTIME_IMAGE_HOST_GATE_OK"
