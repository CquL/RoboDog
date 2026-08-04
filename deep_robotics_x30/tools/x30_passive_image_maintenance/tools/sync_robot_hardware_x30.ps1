# 将已审查的 X30-only HAL 子集复制到 Docker 构建上下文，并生成确定性的
# 完整性清单。目标目录属于生成数据，应先修改 robot_hardware，再运行本脚本。
param(
    [string]$SourceRoot,
    [string]$DestinationRoot
)

$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$workspaceRoot = (Resolve-Path (Join-Path $projectRoot "..")).Path
$transferRoot = Join-Path `
    $projectRoot "docker\x30_livox_ros2_transfer"

if ([string]::IsNullOrWhiteSpace($SourceRoot)) {
    $SourceRoot = Join-Path $workspaceRoot "robot_hardware\robot_hardware"
}
if ([string]::IsNullOrWhiteSpace($DestinationRoot)) {
    $DestinationRoot = Join-Path $transferRoot "components\robot_hardware_x30"
}

$SourceRoot = (Resolve-Path $SourceRoot).Path
New-Item -ItemType Directory -Force -Path $DestinationRoot | Out-Null
$DestinationRoot = (Resolve-Path $DestinationRoot).Path

# 此白名单必须保持精简：被动 X30 镜像不能混入 Unitree、仅供 ROS1 使用的
# 二进制文件、构建输出或无关实验代码。
$files = @(
    "CMakeLists.txt",
    "README.md",
    "cmake_uninstall.cmake.in",
    "config.yaml",
    "robot_hardwareConfig.cmake.in",
    "robot_test_x30.cpp",
    "include/deep_robotics/deep_robotics_x30.h",
    "include/deep_robotics/x30_udp_protocol.h",
    "include/robot_factory.h",
    "include/robot_hardware_constant.h",
    "include/robot_hardware_error_code.h",
    "include/robot_hardware_interface.h",
    "src/deep_robotics/deep_robotics_x30.cpp",
    "src/robot_hardware_constant.cpp",
    "tests/x30_factory_contract_test.cpp",
    "tests/x30_udp_protocol_test.cpp"
)

# 删除旧版拆分测试程序的文件名，防止迁移到统一 robot_test_x30 后，
# 过期测试程序又重新进入部署包。
$retiredFiles = @(
    "robot_test_x30_udp.cpp",
    "robot_test_x30_udp_odom.cpp",
    "robot_test_x30_gait.cpp",
    "robot_test_x30_stair.cpp"
)
foreach ($relativePath in $retiredFiles) {
    $retiredPath = Join-Path $DestinationRoot $relativePath
    if (Test-Path -LiteralPath $retiredPath -PathType Leaf) {
        Remove-Item -LiteralPath $retiredPath -Force
    }
}

foreach ($relativePath in $files) {
    $source = Join-Path $SourceRoot $relativePath
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "Missing robot_hardware source file: $source"
    }

    $destination = Join-Path $DestinationRoot $relativePath
    $destinationDirectory = Split-Path -Parent $destination
    New-Item -ItemType Directory -Force -Path $destinationDirectory |
        Out-Null
    Copy-Item -LiteralPath $source -Destination $destination -Force
}

# Docker 构建会在编译内置 HAL 前校验 SHA256SUMS，使不完整复制或手工修改的
# 快照直接失败，不继续生成存在歧义的镜像。
$manifestPath = Join-Path $DestinationRoot "SHA256SUMS"
if (Test-Path -LiteralPath $manifestPath) {
    Remove-Item -LiteralPath $manifestPath -Force
}

$manifestLines = foreach ($file in Get-ChildItem -LiteralPath $DestinationRoot `
        -Recurse -File | Sort-Object FullName) {
    $relativePath = $file.FullName.Substring($DestinationRoot.Length + 1).
        Replace("\", "/")
    $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).
        Hash.ToLowerInvariant()
    "$hash  $relativePath"
}

$manifestText = ($manifestLines -join "`n") + "`n"
$utf8WithoutBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText(
    $manifestPath,
    $manifestText,
    $utf8WithoutBom
)

Write-Output "Synchronized X30 HAL component:"
Write-Output "  source:      $SourceRoot"
Write-Output "  destination: $DestinationRoot"
Write-Output "  files:       $($files.Count)"
Write-Output "  manifest:    $manifestPath"
