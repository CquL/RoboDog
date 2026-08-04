# Purpose: fetch pinned Unitree SDK, ROS interface, model, and simulator sources.
# Input: optional Destination and official unitreerobotics GitHub ZIP archives.
# Output: cached ZIPs, vendor snapshots, per-repository .source.json files, and
#         OFFICIAL_SOURCE_MANIFEST.json.
# Safety: only pinned commits are accepted; unknown vendor directories are not
# overwritten, and temporary cleanup is constrained to the vendor root.
param(
    [string]$Destination = (Join-Path $PSScriptRoot "..")
)

# Stop on any download, extraction, hash, or manifest-write failure.
$ErrorActionPreference = "Stop"

# Normalize the destination and create the cache and vendor roots.
$root = [System.IO.Path]::GetFullPath($Destination)
$downloadRoot = Join-Path $root "downloads"
$vendorRoot = Join-Path $root "vendor"
[System.IO.Directory]::CreateDirectory($downloadRoot) | Out-Null
[System.IO.Directory]::CreateDirectory($vendorRoot) | Out-Null

# This allowlist and its commits are immutable build/provenance inputs.
$repos = @(
    @{
        name = "unitree_sdk2"
        commit = "21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b"
        role = "Required C++ SDK2 with H2 high-level and low-level examples"
    },
    @{
        name = "unitree_sdk2_python"
        commit = "e4cd91f051aaa77a70600e3d2bf7f50889db1980"
        role = "Optional Python SDK2 prototype interface"
    },
    @{
        name = "unitree_ros2"
        commit = "668d1ec5a05d1c38d3306bdca7d59f2ba3581a88"
        role = "ROS 2 IDL, examples and official CycloneDDS container reference"
    },
    @{
        name = "unitree_ros"
        commit = "d96d8f63ae17a7108d4f7229c00ef875ba7129c9"
        role = "ROS 1 simulation repository containing H2 URDF and meshes; not the H2 real-robot control base"
    },
    @{
        name = "unitree_model"
        commit = "b6a8942b0803b6c137e58cef12beb4b03e4a2fa7"
        role = "Official H2/H2 Plus USD robot models"
    },
    @{
        name = "unitree_mujoco"
        commit = "ae6a8403e272733e9996ef59990880330496177f"
        role = "Optional H2 MuJoCo low-level simulation environment"
    }
)

# Per repository: download/cache check, safe extraction, then provenance write.
$results = @()
foreach ($repo in $repos) {
    $name = $repo.name
    $commit = $repo.commit
    $sourceUrl = "https://github.com/unitreerobotics/$name"
    $archiveUrl = "$sourceUrl/archive/$commit.zip"
    $archivePath = Join-Path $downloadRoot "$name-$commit.zip"
    $vendorPath = Join-Path $vendorRoot $name
    $sourceMetadataPath = Join-Path $vendorPath ".source.json"

    # Preserve a cached archive; download only when it is absent.
    if (-not (Test-Path -LiteralPath $archivePath)) {
        Write-Host "Downloading $name at $commit ..."
        Invoke-WebRequest -UseBasicParsing -Uri $archiveUrl -OutFile $archivePath -TimeoutSec 600
    }

    if ((Get-Item -LiteralPath $archivePath).Length -le 0) {
        throw "Downloaded archive is empty: $archivePath"
    }

    # Existing vendor content must carry matching provenance metadata.
    # Refuse to replace untracked content to protect manually managed files.
    if (Test-Path -LiteralPath $vendorPath) {
        if (-not (Test-Path -LiteralPath $sourceMetadataPath)) {
            throw "Refusing to replace an untracked vendor directory: $vendorPath"
        }
        $existing = Get-Content -Raw -LiteralPath $sourceMetadataPath | ConvertFrom-Json
        if ($existing.commit -ne $commit) {
            throw "Vendor directory $vendorPath contains commit $($existing.commit), expected $commit"
        }
    } else {
        # Extract under vendor first; move only after validating the ZIP layout.
        $extractRoot = Join-Path $vendorRoot ".extract-$name-$commit"
        if (Test-Path -LiteralPath $extractRoot) {
            $resolvedExtract = [System.IO.Path]::GetFullPath($extractRoot)
            if (-not $resolvedExtract.StartsWith($vendorRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
                throw "Unsafe extraction cleanup path: $resolvedExtract"
            }
            Remove-Item -LiteralPath $resolvedExtract -Recurse -Force
        }

        Expand-Archive -LiteralPath $archivePath -DestinationPath $extractRoot -Force
        $extracted = Get-ChildItem -LiteralPath $extractRoot -Directory
        if ($extracted.Count -ne 1) {
            throw "Unexpected archive layout for $name"
        }
        Move-Item -LiteralPath $extracted[0].FullName -Destination $vendorPath
        Remove-Item -LiteralPath $extractRoot -Recurse -Force
    }

    # Record hash, size, and official URLs for later offline build audits.
    $hash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash
    $metadata = [ordered]@{
        official_owner = "unitreerobotics"
        repository = $name
        source_url = $sourceUrl
        archive_url = $archiveUrl
        commit = $commit
        role = $repo.role
        retrieved_at = (Get-Date).ToUniversalTime().ToString("o")
        archive_file = [System.IO.Path]::GetFileName($archivePath)
        archive_bytes = (Get-Item -LiteralPath $archivePath).Length
        archive_sha256 = $hash
        license_file = "LICENSE"
    }

    $metadataJson = $metadata | ConvertTo-Json -Depth 5
    [System.IO.File]::WriteAllText(
        $sourceMetadataPath,
        $metadataJson + [Environment]::NewLine,
        [System.Text.UTF8Encoding]::new($false))
    $results += [pscustomobject]$metadata
}

# The aggregate manifest indexes this run; it does not replace .source.json.
$manifestPath = Join-Path $root "OFFICIAL_SOURCE_MANIFEST.json"
$manifestJson = $results | ConvertTo-Json -Depth 6
[System.IO.File]::WriteAllText(
    $manifestPath,
    $manifestJson + [Environment]::NewLine,
    [System.Text.UTF8Encoding]::new($false))

Write-Host "Official source manifest: $manifestPath"
$results | Select-Object repository, commit, archive_bytes, archive_sha256 | Format-Table -AutoSize
