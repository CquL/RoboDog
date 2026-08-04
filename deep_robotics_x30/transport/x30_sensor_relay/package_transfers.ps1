[CmdletBinding()]
param(
    [ValidateSet("all", "105", "106")]
    [string]$Target = "all"
)

$ErrorActionPreference = "Stop"

# 按主机分别打包：105 只接收 ROS1 转发程序，106 接收被动 ROS2 镜像源码。
# build/devel 等缓存不进入迁移包。
$RelayRoot = $PSScriptRoot
$X30Root = (Resolve-Path (Join-Path $RelayRoot "..\..")).Path

$Packages = @(
    @{
        Target = "105"
        Source = Join-Path $RelayRoot "x30_sensor_forwarder_105"
        Output = Join-Path $X30Root "artifacts\x30_sensor_forwarder_105.tar.gz"
    },
    @{
        Target = "106"
        Source = Join-Path $X30Root "docker\x30_livox_ros2_transfer"
        Output = Join-Path $X30Root "artifacts\x30_livox_ros2_transfer.tar.gz"
    }
)

foreach ($Package in $Packages) {
    if ($Target -ne "all" -and $Package.Target -ne $Target) {
        continue
    }

    $Source = (Resolve-Path $Package.Source).Path
    $SourceParent = Split-Path -Parent $Source
    $SourceName = Split-Path -Leaf $Source
    $Output = $Package.Output
    $OutputDirectory = Split-Path -Parent $Output
    $Temporary = "${Output}.tmp"

    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    Remove-Item -LiteralPath $Temporary -Force -ErrorAction SilentlyContinue

    Write-Host "[package] source: $Source"
    Write-Host "[package] output: $Output"

    Push-Location $SourceParent
    try {
        # 先在目标旁生成临时包，tar 成功后再替换，避免失败时破坏上一份可用包。
        & tar -czf $Temporary `
            --exclude=".git" `
            --exclude=".pytest_cache" `
            --exclude="__pycache__" `
            --exclude="*.pyc" `
            --exclude="build" `
            --exclude="devel" `
            --exclude="install" `
            --exclude="log" `
            $SourceName
        if ($LASTEXITCODE -ne 0) {
            throw "tar failed for $Source"
        }
    }
    finally {
        Pop-Location
    }

    Move-Item -LiteralPath $Temporary -Destination $Output -Force

    # 使用 ASCII LF，兼容 Ubuntu 的 sha256sum -c，并避免部分 PowerShell
    # 文本输出命令产生 CRLF 文件名问题。
    $Hash = Get-FileHash -Algorithm SHA256 -LiteralPath $Output
    $HashLine = "{0}  {1}" -f $Hash.Hash.ToLowerInvariant(),
        (Split-Path -Leaf $Output)
    [System.IO.File]::WriteAllText(
        "${Output}.sha256",
        $HashLine + "`n",
        [System.Text.Encoding]::ASCII
    )

    Write-Host "[package] bytes: $((Get-Item -LiteralPath $Output).Length)"
    Write-Host "[package] SHA256: $($Hash.Hash)"
}
