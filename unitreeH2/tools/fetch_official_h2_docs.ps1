# Purpose: snapshot the Chinese and English Unitree H2 developer documentation.
# Input: optional Destination, the Unitree catalog API, and catalog page URLs.
# Output: locale pages, catalog.json files, a hash manifest, and snapshot notice.
# Safety: downloaded pages are data only; the snapshot is not immutable API
# truth and does not imply redistribution rights or an official PDF manual.
param(
    [string]$Destination = (Join-Path $PSScriptRoot "..")
)

# Stop on network, catalog, or file-write failure to avoid partial manifests.
$ErrorActionPreference = "Stop"

# Keep all generated documentation below unitreeH2/official_docs.
$root = [System.IO.Path]::GetFullPath($Destination)
$docsRoot = Join-Path $root "official_docs"
[System.IO.Directory]::CreateDirectory($docsRoot) | Out-Null

# Convert a catalog path to a cross-platform filename with stable fallbacks.
function Convert-ToSafeName([string]$name) {
    if ([string]::IsNullOrWhiteSpace($name)) {
        return "index"
    }
    $safe = $name -replace '[^A-Za-z0-9._-]', '_'
    $safe = $safe.Trim('_')
    if ([string]::IsNullOrWhiteSpace($safe)) {
        return "page"
    }
    return $safe
}

# Flatten the catalog tree while retaining parent entries and directory order.
function Get-DirectoryEntries($nodes) {
    $all = @()
    foreach ($node in @($nodes)) {
        $all += $node
        if ($node.children -and @($node.children).Count -gt 0) {
            $all += Get-DirectoryEntries $node.children
        }
    }
    return $all
}

# Store Chinese and English snapshots separately to prevent name collisions.
$results = @()
foreach ($locale in @("zh", "en")) {
    $localeRoot = Join-Path $docsRoot $locale
    [System.IO.Directory]::CreateDirectory($localeRoot) | Out-Null

    # The catalog supplies hierarchy, path, update time, and content URL.
    $catalogUrl = "https://robot-api.unitree.com/doc?space=H2_developer&locale=$locale"
    $catalogResponse = Invoke-RestMethod -Uri $catalogUrl -TimeoutSec 60
    if ($catalogResponse.code -ne 100) {
        throw "H2 document catalog request failed for locale $locale"
    }

    $catalogPath = Join-Path $localeRoot "catalog.json"
    $catalogJson = $catalogResponse.data | ConvertTo-Json -Depth 30
    [System.IO.File]::WriteAllText(
        $catalogPath,
        $catalogJson + [Environment]::NewLine,
        [System.Text.UTF8Encoding]::new($false))

    # Save pages in stable order and record source, time, size, and SHA256.
    $entries = Get-DirectoryEntries $catalogResponse.data.directory
    $index = 0
    foreach ($entry in $entries) {
        if ([string]::IsNullOrWhiteSpace($entry.url)) {
            continue
        }

        $index += 1
        $safePath = Convert-ToSafeName $entry.path
        $pagePath = Join-Path $localeRoot ("{0:D2}_{1}.md" -f $index, $safePath)
        $response = Invoke-WebRequest -UseBasicParsing -Uri $entry.url -TimeoutSec 60
        $bytes = [byte[]]$response.Content
        [System.IO.File]::WriteAllBytes($pagePath, $bytes)

        $results += [pscustomobject]@{
            locale = $locale
            path = $entry.path
            name = $entry.name
            source_url = $entry.url
            public_page_url = "https://support.unitree.com/home/$locale/H2_developer/$($entry.path)"
            official_update_time = $entry.updateTime
            retrieved_at = (Get-Date).ToUniversalTime().ToString("o")
            local_file = "official_docs/$locale/$([System.IO.Path]::GetFileName($pagePath))"
            bytes = (Get-Item -LiteralPath $pagePath).Length
            sha256 = (Get-FileHash -LiteralPath $pagePath -Algorithm SHA256).Hash
        }
    }
}

# The aggregate manifest supports page-by-page comparison on later refreshes.
$manifestPath = Join-Path $docsRoot "H2_OFFICIAL_DOCS_MANIFEST.json"
$manifestJson = $results | ConvertTo-Json -Depth 8
[System.IO.File]::WriteAllText(
    $manifestPath,
    $manifestJson + [Environment]::NewLine,
    [System.Text.UTF8Encoding]::new($false))

# The notice states the snapshot and licensing boundary explicitly.
$notice = @(
    "# Unitree H2 official online-document snapshot",
    "",
    "- Source: Unitree Documentation Center, H2 SDK Development Guide.",
    "- Retrieved at: $((Get-Date).ToUniversalTime().ToString('o')).",
    "- This is a local engineering snapshot. The web pages do not declare an open redistribution license.",
    "- No public H2/H2 EDU user-manual or data-sheet PDF was found in the official download center.",
    "- Re-run the fetch script and compare the manifest; do not treat this snapshot as immutable firmware/API truth."
) -join [Environment]::NewLine
[System.IO.File]::WriteAllText(
    (Join-Path $docsRoot "README.md"),
    $notice,
    [System.Text.UTF8Encoding]::new($false))

Write-Host "H2 document manifest: $manifestPath"
$results | Group-Object locale | Select-Object Name, Count | Format-Table -AutoSize
