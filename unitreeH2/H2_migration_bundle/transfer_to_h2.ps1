[CmdletBinding()]
param(
    [string]$HostAddress = "192.168.123.162",
    [string]$UserName = "unitree",
    [string]$RemoteParent = "/home/unitree/p2_unitreeH2",
    [switch]$VerifyOnly
)

# Windows-side helper. It verifies both large archives before copying the
# complete self-contained folder to an H2 PC2 over SSH/SCP.
$ErrorActionPreference = "Stop"

if ($HostAddress -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$') {
    throw "Invalid HostAddress: $HostAddress"
}
if ($UserName -notmatch '^[A-Za-z0-9][A-Za-z0-9_.-]*$') {
    throw "Invalid UserName: $UserName"
}
if ($RemoteParent -notmatch '^/[A-Za-z0-9_./-]+$') {
    throw "Invalid RemoteParent: $RemoteParent"
}
if (($RemoteParent -split '/') -contains '..') {
    throw "RemoteParent must not contain '..': $RemoteParent"
}

$bundleRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$bundleName = Split-Path -Leaf $bundleRoot
if ($bundleName -notmatch '^[A-Za-z0-9][A-Za-z0-9_.-]*$') {
    throw "Invalid bundle directory name: $bundleName"
}

function Assert-BundleManifest {
    param([Parameter(Mandatory = $true)][string]$Root)

    $manifest = Join-Path $Root 'SHA256SUMS'
    if (-not (Test-Path -LiteralPath $manifest -PathType Leaf)) {
        throw "Missing bundle manifest: $manifest"
    }

    $rootPrefix = [System.IO.Path]::GetFullPath(
        $Root + [System.IO.Path]::DirectorySeparatorChar
    )
    $manifestPath = [System.IO.Path]::GetFullPath($manifest)
    $listed = @{}
    $checked = 0
    foreach ($line in Get-Content -LiteralPath $manifest -Encoding UTF8) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        if ($line -notmatch '^([0-9A-Fa-f]{64})  \./(.+)$') {
            throw "Invalid SHA256SUMS line: $line"
        }

        $expected = $Matches[1].ToLowerInvariant()
        $relative = $Matches[2]
        if (($relative -split '/') -contains '..') {
            throw "Unsafe manifest path: $relative"
        }
        $localPath = [System.IO.Path]::GetFullPath(
            (Join-Path $Root ($relative -replace '/', '\'))
        )
        if (-not $localPath.StartsWith(
                $rootPrefix,
                [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Manifest path escapes bundle: $relative"
        }
        if (-not (Test-Path -LiteralPath $localPath -PathType Leaf)) {
            throw "Manifest file is missing: $relative"
        }

        $actual = (Get-FileHash -LiteralPath $localPath -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actual -ne $expected) {
            throw "SHA256 mismatch: $relative expected=$expected actual=$actual"
        }
        $listed[$localPath.ToLowerInvariant()] = $true
        $checked++
    }
    if ($checked -eq 0) {
        throw 'SHA256SUMS contains no files.'
    }

    foreach ($file in Get-ChildItem -LiteralPath $Root -File -Recurse -Force) {
        $fullPath = [System.IO.Path]::GetFullPath($file.FullName)
        if ($fullPath.Equals(
                $manifestPath,
                [System.StringComparison]::OrdinalIgnoreCase)) {
            continue
        }
        if (-not $listed.ContainsKey($fullPath.ToLowerInvariant())) {
            $extra = $fullPath.Substring($rootPrefix.Length).Replace('\', '/')
            throw "Unlisted file in migration bundle: $extra"
        }
    }
    Write-Host "BUNDLE_SHA256_OK files=$checked"
}

# Target-specific runtime state must never be copied to another robot.
if (Test-Path -LiteralPath (Join-Path $bundleRoot 'config\deployment.env')) {
    throw 'Remove or move config\deployment.env before migration; create it on the target H2 instead.'
}
if (Test-Path -LiteralPath (Join-Path $bundleRoot 'state')) {
    throw 'Remove or move the generated state directory before migration.'
}
Assert-BundleManifest -Root $bundleRoot
if ($VerifyOnly) {
    Write-Host 'LOCAL_BUNDLE_VERIFY_ONLY_OK'
    return
}

$target = "${UserName}@${HostAddress}"
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$remoteFinal = "${RemoteParent}/${bundleName}"
$remoteStage = "${RemoteParent}/${bundleName}.upload-${stamp}"
$prepare = "mkdir -p -- '$RemoteParent' && test ! -e '$remoteFinal' && test ! -e '$remoteStage'"

& ssh $target $prepare
if ($LASTEXITCODE -ne 0) {
    throw "Remote target already exists or cannot be prepared: $remoteFinal"
}

& scp -r $bundleRoot "${target}:$remoteStage"
if ($LASTEXITCODE -ne 0) {
    throw "SCP failed with exit code $LASTEXITCODE; inspect $remoteStage on the target."
}

& ssh $target "cd -- '$remoteStage' && bash h2_bundle.sh verify"
if ($LASTEXITCODE -ne 0) {
    throw "Remote SHA256 verification failed; staged files were left at $remoteStage for inspection."
}

& ssh $target "mv -- '$remoteStage' '$remoteFinal'"
if ($LASTEXITCODE -ne 0) {
    throw "Remote finalization failed; staged files remain at $remoteStage."
}

Write-Host "TRANSFER_AND_REMOTE_VERIFY_OK target=${target}:${remoteFinal}"
Write-Host "Next: cd ${remoteFinal} && cp config/deployment.env.example config/deployment.env"
