# 从同一个已验证工作区生成三份可迁移的 X30 部署包。
# 本脚本不会构建 Docker 镜像，也不会连接机器狗。
[CmdletBinding()]
param(
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"

# 所有输入路径都从脚本位置解析，避免打包结果依赖调用者当前目录。
$X30Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$WorkspaceRoot = (Resolve-Path (Join-Path $X30Root "..")).Path
$ArtifactsRoot = Join-Path $X30Root "artifacts"
$RelayPackager = Join-Path `
    $X30Root "transport\x30_sensor_relay\package_transfers.ps1"
$HalSync = Join-Path $X30Root `
    "tools\x30_passive_image_maintenance\tools\sync_robot_hardware_x30.ps1"
$HalTests = Join-Path $X30Root `
    "tools\x30_passive_image_maintenance\tests"
$RobotHardwareTests = Join-Path `
    $WorkspaceRoot "robot_hardware\robot_hardware\tests"
$RobotHardwareRelative = "robot_hardware/robot_hardware"
$RobotHardwareArchive = Join-Path `
    $ArtifactsRoot "robot_hardware_x30_udp_transfer.tar.gz"

New-Item -ItemType Directory -Force -Path $ArtifactsRoot | Out-Null

Write-Host "[artifacts] X30 root: $X30Root"
Write-Host "[artifacts] output:   $ArtifactsRoot"

# Docker 镜像中包含 X30-only HAL 源码快照。测试和打包前先刷新快照，
# 防止宿主机源码与容器内源码在不知情的情况下发生偏差。
Write-Host "[artifacts] synchronizing the Docker X30 HAL snapshot..."
& $HalSync

# 两组测试都是合同测试：一组保护 UDP/HAL 行为，另一组保护精简后的
# 被动接收镜像布局。
if (-not $SkipTests) {
    Write-Host "[artifacts] running robot_hardware tests..."
    & python -m pytest -q $RobotHardwareTests
    if ($LASTEXITCODE -ne 0) {
        throw "robot_hardware tests failed"
    }

    Write-Host "[artifacts] running passive-image contract tests..."
    & python -m pytest -q $HalTests
    if ($LASTEXITCODE -ne 0) {
        throw "passive-image contract tests failed"
    }
}
else {
    Write-Warning "Tests were skipped by explicit request."
}

Write-Host "[artifacts] packaging 105 and 106 transfer bundles..."
& $RelayPackager -Target all

# 先用临时文件名生成压缩包，只有 tar 成功后才覆盖正式产物。
# 因此打包失败时，上一份完整可用的压缩包仍会保留。
$RobotHardwareTemporary = "${RobotHardwareArchive}.tmp"
if (Test-Path -LiteralPath $RobotHardwareTemporary) {
    Remove-Item -LiteralPath $RobotHardwareTemporary -Force
}

Write-Host "[artifacts] packaging robot_hardware ROS1 test sources..."
Push-Location $WorkspaceRoot
try {
    & tar -czf $RobotHardwareTemporary `
        --exclude=".git" `
        --exclude=".pytest_cache" `
        --exclude="__pycache__" `
        --exclude="*.pyc" `
        --exclude="build" `
        --exclude="build_x30" `
        --exclude="devel" `
        --exclude="install" `
        --exclude="log" `
        $RobotHardwareRelative
    if ($LASTEXITCODE -ne 0) {
        throw "tar failed for $RobotHardwareRelative"
    }
}
finally {
    Pop-Location
}

Move-Item -LiteralPath $RobotHardwareTemporary `
    -Destination $RobotHardwareArchive -Force

# 使用兼容 GNU sha256sum 的文本格式保存摘要，使同一个校验文件既能在
# 上传前由 PowerShell 使用，也能在上传后由 Ubuntu 使用。
$RobotHardwareHash = Get-FileHash `
    -Algorithm SHA256 -LiteralPath $RobotHardwareArchive
$RobotHardwareHashLine = "{0}  {1}" -f `
    $RobotHardwareHash.Hash.ToLowerInvariant(),
    (Split-Path -Leaf $RobotHardwareArchive)
[System.IO.File]::WriteAllText(
    "${RobotHardwareArchive}.sha256",
    $RobotHardwareHashLine + "`n",
    [System.Text.Encoding]::ASCII
)

$RequiredArchives = @(
    "x30_sensor_forwarder_105.tar.gz",
    "x30_livox_ros2_transfer.tar.gz",
    "robot_hardware_x30_udp_transfer.tar.gz"
)

# 校验文件缺失或摘要不一致都视为打包失败，不能把来源不明确的包交给实机。
foreach ($ArchiveName in $RequiredArchives) {
    $ArchivePath = Join-Path $ArtifactsRoot $ArchiveName
    $HashPath = "${ArchivePath}.sha256"
    if (-not (Test-Path -LiteralPath $ArchivePath -PathType Leaf)) {
        throw "Missing deployment archive: $ArchivePath"
    }
    if (-not (Test-Path -LiteralPath $HashPath -PathType Leaf)) {
        throw "Missing deployment hash: $HashPath"
    }

    $ExpectedHash = (
        Get-Content -LiteralPath $HashPath -Raw
    ).Trim().Split(" ")[0].ToUpperInvariant()
    $ActualHash = (
        Get-FileHash -Algorithm SHA256 -LiteralPath $ArchivePath
    ).Hash
    if ($ExpectedHash -ne $ActualHash) {
        throw "SHA256 mismatch for $ArchiveName"
    }
}

$ImageArchive = Join-Path `
    $ArtifactsRoot "x30_livox_ros2_jezetek_amd64.tar"
$ImageHash = "${ImageArchive}.sha256"

# 最终镜像依赖 106 上已经导入的 AMD64 基础镜像，因此只能由已验证的
# 106 主机导出；本机仅在镜像文件存在时校验其摘要。
if (Test-Path -LiteralPath $ImageArchive -PathType Leaf) {
    if (-not (Test-Path -LiteralPath $ImageHash -PathType Leaf)) {
        throw "Docker image archive exists without its SHA256 file: $ImageHash"
    }
    $ExpectedImageHash = (
        Get-Content -LiteralPath $ImageHash -Raw
    ).Trim().Split(" ")[0].ToUpperInvariant()
    $ActualImageHash = (
        Get-FileHash -Algorithm SHA256 -LiteralPath $ImageArchive
    ).Hash
    if ($ExpectedImageHash -ne $ActualImageHash) {
        throw "SHA256 mismatch for the Docker image archive"
    }
}
else {
    Write-Warning (
        "Final Docker image archive is not present. Rebuild/export it on " +
        "verified host 106 after Docker sources change."
    )
}

Write-Host "[artifacts] current deployment files:"
Get-ChildItem -LiteralPath $ArtifactsRoot -File |
    Sort-Object Name |
    Select-Object Name, Length, LastWriteTime
