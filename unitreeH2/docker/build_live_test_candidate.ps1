param(
    [string]$Image = "unitree_h2:amd64-live-test-candidate",
    [string]$BaseImage = "jezetek:navigation_system_amd64",
    [string]$SdkCommit = "21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b",
    [switch]$NoCache
)

$ErrorActionPreference = "Stop"

$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $unitreeH2Root
$sdkDirectory = Join-Path $unitreeH2Root "vendor/unitree_sdk2"
$halDirectory = Join-Path $repoRoot "robot_hardware/robot_hardware"
$dockerfile = Join-Path $PSScriptRoot "Dockerfile"

foreach ($path in @($sdkDirectory, $halDirectory, $dockerfile)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Missing build input: $path"
    }
}

$sourceMetadataPath = Join-Path $sdkDirectory ".source.json"
$sourceMetadata = Get-Content -LiteralPath $sourceMetadataPath -Raw |
    ConvertFrom-Json
if ($sourceMetadata.commit -ne $SdkCommit) {
    throw "SDK2 commit mismatch: expected $SdkCommit, got $($sourceMetadata.commit)"
}

docker image inspect $BaseImage *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Missing local base image: $BaseImage"
}

$buildArgs = @("buildx", "build")
if ($NoCache) {
    $buildArgs += "--no-cache"
}
$buildArgs += @(
    "--load",
    "--pull=false",
    "--network=none",
    "--build-arg", "BASE_IMAGE=$BaseImage",
    "--build-arg", "UNITREE_SDK2_COMMIT=$SdkCommit",
    "--build-context", "sdk2=$sdkDirectory",
    "--build-context", "robot_hardware=$halDirectory",
    "--file", $dockerfile,
    "--tag", $Image,
    $PSScriptRoot
)
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
& docker @buildArgs
$buildExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($buildExitCode -ne 0) {
    throw "H2 live-test candidate build failed with exit code $buildExitCode"
}

$inspect = (docker image inspect $Image | ConvertFrom-Json)[0]
if ($inspect.Architecture -ne "amd64") {
    throw "Unexpected image architecture: $($inspect.Architecture)"
}
if ($inspect.Config.Labels.'io.robodog.unitree_sdk2.commit' -ne $SdkCommit) {
    throw "Built image SDK2 label mismatch"
}
if ($inspect.Config.Entrypoint -and $inspect.Config.Entrypoint.Count -gt 0) {
    throw "Built image unexpectedly inherited an entrypoint"
}
if ($inspect.Config.Env | Where-Object { $_ -like "RMW_IMPLEMENTATION=*" }) {
    throw "Built image unexpectedly selects a ROS 2 RMW implementation"
}

Write-Host "IMAGE=$Image"
Write-Host "IMAGE_ID=$($inspect.Id)"
Write-Host "ARCHITECTURE=$($inspect.Architecture)"
Write-Host "SDK2_COMMIT=$SdkCommit"
Write-Host "H2_LIVE_TEST_CANDIDATE_BUILD_OK"
