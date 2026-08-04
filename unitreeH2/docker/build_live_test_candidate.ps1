# Purpose: build an H2 HAL live-test candidate from pinned SDK2 and current HAL.
# Input: image names, SDK commit, and optional NoCache.
# Output: local image plus its ID, architecture, and SDK provenance label.
# Safety: build is offline/no-pull and does not run a container or robot command.
# The result must not inherit an entrypoint or preselect a ROS 2 RMW.
param(
    [string]$Image = "unitree_h2:amd64-live-test-candidate",
    [string]$BaseImage = "jezetek:navigation_system_amd64",
    [string]$SdkCommit = "21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b",
    [switch]$NoCache
)

$ErrorActionPreference = "Stop"

# Resolve all local inputs from the script path, not the caller directory.
$unitreeH2Root = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $unitreeH2Root
$sdkDirectory = Join-Path $unitreeH2Root "vendor/unitree_sdk2"
$halDirectory = Join-Path $repoRoot "robot_hardware/robot_hardware"
$dockerfile = Join-Path $PSScriptRoot "Dockerfile"

# Validate every input before invoking Docker.
foreach ($path in @($sdkDirectory, $halDirectory, $dockerfile)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Missing build input: $path"
    }
}

# Match .source.json against the requested commit to prevent false labels.
$sourceMetadataPath = Join-Path $sdkDirectory ".source.json"
$sourceMetadata = Get-Content -LiteralPath $sourceMetadataPath -Raw |
    ConvertFrom-Json
if ($sourceMetadata.commit -ne $SdkCommit) {
    throw "SDK2 commit mismatch: expected $SdkCommit, got $($sourceMetadata.commit)"
}

# Require a preloaded base image; the script will not pull it.
docker image inspect $BaseImage *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Missing local base image: $BaseImage"
}

# NoCache changes cache use only; offline and local-context boundaries remain.
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
# Capture Docker's native exit code, then restore the PowerShell error policy.
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
& docker @buildArgs
$buildExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($buildExitCode -ne 0) {
    throw "H2 live-test candidate build failed with exit code $buildExitCode"
}

# Verify architecture, provenance, entrypoint, and RMW state after the build.
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
