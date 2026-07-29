param(
    [string]$Image = "unitree_h2:amd64-runtime-candidate",
    [string]$BaseImage = "jezetek:navigation_system_amd64"
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..\..")).Path
$sdkDir = Join-Path $repoRoot "unitreeH2\vendor\unitree_sdk2"
$halDir = Join-Path $repoRoot "robot_hardware\robot_hardware"
$stateProbeDir = Join-Path $repoRoot "unitreeH2\remote\05_hg_state_probe"
$sensorBridgeDir = Join-Path $repoRoot "unitreeH2\sensor_bridge"

if (-not (Test-Path (Join-Path $sdkDir ".source.json"))) {
    throw "Missing pinned Unitree SDK2 snapshot: $sdkDir"
}
if (-not (Test-Path (Join-Path $stateProbeDir "h2_hg_state_read_only_probe.cpp"))) {
    throw "Missing H2 HG state probe source: $stateProbeDir"
}
if (-not (Test-Path (Join-Path $sensorBridgeDir "src\unitree_h2_sensor_bridge.cpp"))) {
    throw "Missing H2 sensor bridge source: $sensorBridgeDir"
}

docker info --format '{{.ServerVersion}}' | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Docker engine is not ready" }
docker image inspect $BaseImage | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Base image is not loaded: $BaseImage" }

docker buildx build `
    --load `
    --pull=false `
    --network=none `
    --build-arg "BASE_IMAGE=$BaseImage" `
    --build-context "sdk2=$sdkDir" `
    --build-context "robot_hardware=$halDir" `
    --build-context "h2_state_probe=$stateProbeDir" `
    --build-context "h2_sensor_bridge=$sensorBridgeDir" `
    --file (Join-Path $scriptDir "Dockerfile.runtime") `
    --tag $Image `
    $scriptDir
if ($LASTEXITCODE -ne 0) { throw "H2 runtime image build failed" }

& (Join-Path $scriptDir "verify_h2_runtime_image_offline.ps1") -Image $Image
if ($LASTEXITCODE -ne 0) { throw "H2 runtime image verification failed" }

$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
Write-Host "IMAGE=$Image"
Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "IMAGE_ARCH=$($inspect.Architecture)"
Write-Host "H2_RUNTIME_IMAGE_BUILD_OK"
