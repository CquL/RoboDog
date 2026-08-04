param(
    [switch]$Apply
)

$ErrorActionPreference = "Stop"

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../.."))
$x30Root = [System.IO.Path]::GetFullPath((Join-Path $repoRoot "deep_robotics_x30"))
$mountainNotesName = ([char]0x5C71).ToString() + ([char]0x5730).ToString() + ".md"

function Assert-InWorkspace([string]$path) {
    $full = [System.IO.Path]::GetFullPath($path)
    if (-not $full.StartsWith($repoRoot + [System.IO.Path]::DirectorySeparatorChar,
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Path escapes workspace: $full"
    }
    return $full
}

function Get-PathStats([string]$path) {
    $item = Get-Item -LiteralPath $path
    if ($item.PSIsContainer) {
        $files = @(Get-ChildItem -LiteralPath $path -File -Recurse -Force -ErrorAction Stop)
        $bytes = ($files | Measure-Object -Property Length -Sum).Sum
        if ($null -eq $bytes) { $bytes = 0 }
        return [ordered]@{
            kind = "directory"
            file_count = $files.Count
            bytes = [int64]$bytes
            sha256 = $null
        }
    }

    return [ordered]@{
        kind = "file"
        file_count = 1
        bytes = [int64]$item.Length
        sha256 = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
    }
}

$moves = @(
    @{ source = "docs"; destination = "deep_robotics_x30/docs" },
    @{ source = "launch"; destination = "deep_robotics_x30/factory/launch" },
    @{ source = "message_transformer_cpp"; destination = "deep_robotics_x30/factory/message_transformer_cpp" },
    @{ source = "x30_plane_seg_bundle"; destination = "deep_robotics_x30/factory/plane_seg_bundle" },
    @{ source = "libgrid_map_transformer.so"; destination = "deep_robotics_x30/factory/binaries/libgrid_map_transformer.so" },
    @{ source = "livox_ros_driver2"; destination = "deep_robotics_x30/sensors/livox/livox_ros_driver2" },
    @{ source = "livox_ros_driver2.zip"; destination = "deep_robotics_x30/sensors/livox/livox_ros_driver2.zip" },
    @{ source = "multi_MID360_config.json"; destination = "deep_robotics_x30/sensors/livox/multi_MID360_config.json" },
    @{ source = "third_party/yesense_ros2_avalue"; destination = "deep_robotics_x30/sensors/yesense/yesense_ros2_avalue" },
    @{ source = "x30_cloud_test"; destination = "deep_robotics_x30/data/cloud_test" },
    @{ source = "x30_gridmap_captures"; destination = "deep_robotics_x30/data/gridmap_captures" },
    @{ source = "x30_stair_baselines"; destination = "deep_robotics_x30/data/stair_baselines" },
    @{ source = "x30_udp_captures"; destination = "deep_robotics_x30/data/udp_captures" },
    @{ source = "x30_livox_ros2_transfer"; destination = "deep_robotics_x30/docker/x30_livox_ros2_transfer" },
    @{ source = "x30_livox_ros2_transfer.tar.gz"; destination = "deep_robotics_x30/artifacts/x30_livox_ros2_transfer.tar.gz" },
    @{ source = "x30_migration_package_20260706"; destination = "deep_robotics_x30/migration/x30_migration_package_20260706" },
    @{ source = "docker_offline_focal_amd64_fix1.tar.gz"; destination = "deep_robotics_x30/migration/docker_offline_focal_amd64_fix1.tar.gz" },
    @{ source = "remote_bridge_probe.txt"; destination = "deep_robotics_x30/logs/remote_bridge_probe.txt" },
    @{ source = "remote_build_yesense_output.txt"; destination = "deep_robotics_x30/logs/remote_build_yesense_output.txt" },
    @{ source = "remote_imu_ros2_ros1_compare.txt"; destination = "deep_robotics_x30/logs/remote_imu_ros2_ros1_compare.txt" },
    @{ source = "remote_yesense_params.txt"; destination = "deep_robotics_x30/logs/remote_yesense_params.txt" },
    @{ source = "remote_yesense_probe.txt"; destination = "deep_robotics_x30/logs/remote_yesense_probe.txt" },
    @{ source = "robot_hardware_x30_udp_transfer.tar.gz"; destination = "deep_robotics_x30/artifacts/robot_hardware_x30_udp_transfer.tar.gz" },
    # 对非 ASCII 手册保留磁盘上的原始文件名。匹配模式只使用 ASCII，
    # 避免 Windows PowerShell 5 在解析脚本时损坏文件名。
    @{ source_pattern = "X30*.pdf"; destination_dir = "deep_robotics_x30/manuals" },
    @{ source_pattern = "*V2.0.4(1).pdf"; destination_dir = "deep_robotics_x30/manuals" },
    @{ source_pattern = "*V2.0.3(1).pdf"; destination_dir = "deep_robotics_x30/manuals" },
    @{ source_pattern = "*ros+udp V1.0.4(1).pdf"; destination_dir = "deep_robotics_x30/manuals" },
    @{ source_pattern = "*.docx"; destination_dir = "deep_robotics_x30/manuals" },
    @{ source = $mountainNotesName; destination_dir = "deep_robotics_x30/manuals" }
)

$records = @()
foreach ($move in $moves) {
    if ($move.ContainsKey("source_pattern")) {
        $matches = @(Get-ChildItem -LiteralPath $repoRoot -File -Force |
            Where-Object { $_.Name -like $move.source_pattern })
        if ($matches.Count -ne 1) {
            throw "Expected exactly one root file for pattern '$($move.source_pattern)', found $($matches.Count)"
        }
        $source = Assert-InWorkspace $matches[0].FullName
    } else {
        $source = Assert-InWorkspace (Join-Path $repoRoot $move.source)
    }

    if ($move.ContainsKey("destination")) {
        $destination = Assert-InWorkspace (Join-Path $repoRoot $move.destination)
    } else {
        $destination = Assert-InWorkspace (Join-Path (Join-Path $repoRoot $move.destination_dir) ([System.IO.Path]::GetFileName($source)))
    }

    if (-not (Test-Path -LiteralPath $source)) {
        throw "Missing source: $source"
    }
    if (Test-Path -LiteralPath $destination) {
        throw "Destination already exists: $destination"
    }

    $before = Get-PathStats $source
    $sourceRelative = $source.Substring($repoRoot.Length + 1)
    $destinationRelative = $destination.Substring($repoRoot.Length + 1)
    $record = [ordered]@{
        source = $sourceRelative
        destination = $destinationRelative
        kind = $before.kind
        file_count = $before.file_count
        bytes = $before.bytes
        sha256 = $before.sha256
        applied = [bool]$Apply
    }

    if ($Apply) {
        [System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($destination)) | Out-Null
        Move-Item -LiteralPath $source -Destination $destination
        $after = Get-PathStats $destination
        if ($after.file_count -ne $before.file_count -or $after.bytes -ne $before.bytes) {
            throw "Post-move verification failed: $destination"
        }
        if ($before.sha256 -and $after.sha256 -ne $before.sha256) {
            throw "Post-move SHA256 mismatch: $destination"
        }
    }

    $records += [pscustomobject]$record
}

$manifest = [ordered]@{
    generated_at = (Get-Date).ToUniversalTime().ToString("o")
    repo_root = $repoRoot
    applied = [bool]$Apply
    preserved_root_items = @("robot_hardware", "jezetek_navigation_amd64.tar")
    moves = $records
}

$manifestName = if ($Apply) { "workspace_move_manifest_20260715.json" } else { "workspace_move_dry_run_20260715.json" }
$manifestPath = Join-Path $x30Root $manifestName
$json = $manifest | ConvertTo-Json -Depth 8
[System.IO.File]::WriteAllText(
    $manifestPath,
    $json + [Environment]::NewLine,
    [System.Text.UTF8Encoding]::new($false))

Write-Host "Manifest: $manifestPath"
$records | Select-Object source, destination, file_count, bytes, applied | Format-Table -AutoSize
