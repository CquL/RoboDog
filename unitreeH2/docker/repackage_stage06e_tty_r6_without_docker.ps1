param(
    [string]$SourceBundle = "",
    [string]$BundleName = "unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r6",
    [string]$OutputDirectory = ""
)

$ErrorActionPreference = "Stop"

function ConvertTo-WslPath([string]$Path) {
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '^[A-Za-z]:\\') {
        throw "Expected a drive-qualified Windows path: $resolved"
    }
    $drive = $resolved.Substring(0, 1).ToLowerInvariant()
    $tail = $resolved.Substring(2).Replace('\', '/')
    return "/mnt/$drive$tail"
}

$unitreeH2Root = Split-Path -Parent $PSScriptRoot
if (-not $SourceBundle) {
    $SourceBundle = Join-Path $unitreeH2Root `
        "runtime_bundle\unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5.tar.gz"
}
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $unitreeH2Root "runtime_bundle"
}

$SourceBundle = (Resolve-Path -LiteralPath $SourceBundle).Path
$remoteDirectory = (Resolve-Path -LiteralPath (Join-Path $unitreeH2Root "remote")).Path
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

$archivePath = Join-Path $OutputDirectory "$BundleName.tar.gz"
$archiveHashPath = "$archivePath.sha256"
if ((Test-Path -LiteralPath $archivePath) -or (Test-Path -LiteralPath $archiveHashPath)) {
    throw "Refusing to overwrite existing artifact: $archivePath"
}

$bash = @'
set -Eeuo pipefail

source_bundle="$1"
remote_dir="$2"
out_dir="$3"
bundle_name="$4"
archive="$out_dir/$bundle_name.tar.gz"
archive_hash="$archive.sha256"
parent_name="unitree_h2_pc2_native_amd64_stage06c_to_06e_20260720_r5"

test -f "$source_bundle"
test -d "$remote_dir"
test ! -e "$archive"
test ! -e "$archive_hash"

work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT

tar -xzf "$source_bundle" -C "$work"
parent="$work/$parent_name"
release="$work/$bundle_name"
test -d "$parent"
mv -- "$parent" "$release"

for relative in \
  08_pc2_h2_single_axis_motion_gate.sh \
  tests/test_h2_gate_schema_offline.sh; do
  install -m 0755 "$remote_dir/$relative" "$release/scripts/$relative"
  sed -i 's/\r$//' "$release/scripts/$relative"
done

parent_sha256="$(sha256sum "$source_bundle" | awk '{print $1}')"
{
  printf 'parent_bundle=%s\n' "$(basename "$source_bundle")"
  printf 'parent_bundle_sha256=%s\n' "$parent_sha256"
  printf 'repackage_reason=stage06e_tty_fd3_fix\n'
  printf 'repackage_time_utc=%s\n' "$(date -u --iso-8601=seconds)"
} >>"$release/meta/build-info.txt"

(
  cd "$release"
  find . -type f ! -path ./meta/manifest.sha256 -print0 |
    sort -z | xargs -0 sha256sum >meta/manifest.sha256
  bash -n scripts/*.sh scripts/tests/*.sh
  TMPDIR="$work" bash scripts/tests/test_h2_gate_schema_offline.sh
)

mkdir -p "$work/parent-copy"
tar -xzf "$source_bundle" -C "$work/parent-copy"
parent_copy="$work/parent-copy/$parent_name"
for tree in bin lib config; do
  diff -r --no-dereference "$parent_copy/$tree" "$release/$tree"
done
cmp "$parent_copy/meta/image-id.txt" "$release/meta/image-id.txt"
cmp "$parent_copy/meta/sdk2-commit.txt" "$release/meta/sdk2-commit.txt"
cmp "$parent_copy/meta/symlinks.txt" "$release/meta/symlinks.txt"

export LD_LIBRARY_PATH="$release/lib"
"$release/bin/unitree_h2_factory_contract_test"
"$release/bin/unitree_h2_direct_api_contract_test"
"$release/bin/unitree_h2_live_motion_plan_test"

mkdir -p "$work/verify"
tar --sort=name --mtime='UTC 2026-07-20 00:00:00' \
  --owner=0 --group=0 --numeric-owner \
  -czf "$archive" -C "$work" "$bundle_name"
(
  cd "$out_dir"
  sha256sum "$bundle_name.tar.gz" >"$bundle_name.tar.gz.sha256"
)

tar -xzf "$archive" -C "$work/verify"
verified="$work/verify/$bundle_name"
(
  cd "$verified"
  sha256sum --check --strict meta/manifest.sha256
  bash -n scripts/*.sh scripts/tests/*.sh
  TMPDIR="$work" bash scripts/tests/test_h2_gate_schema_offline.sh
)
for tree in bin lib config; do
  diff -r --no-dereference "$parent_copy/$tree" "$verified/$tree"
done

printf 'R6_PARENT_SHA256=%s\n' "$parent_sha256"
printf 'R6_BUNDLE_PATH=%s\n' "$archive"
printf 'R6_BUNDLE_SHA256=%s\n' "$(awk '{print $1}' "$archive_hash")"
printf 'R6_TTY_FIX_REPACKAGE_OK\n'
'@

$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($bash))
$args = @(
    "-lc",
    ("echo {0} | base64 -d | bash -s -- '{1}' '{2}' '{3}' '{4}'" -f `
        $encoded,
        (ConvertTo-WslPath $SourceBundle),
        (ConvertTo-WslPath $remoteDirectory),
        (ConvertTo-WslPath $OutputDirectory),
        $BundleName)
)
& bash @args
if ($LASTEXITCODE -ne 0) {
    throw "WSL r6 repackage/verification failed with exit code $LASTEXITCODE"
}

$hash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
$expected = ((Get-Content -LiteralPath $archiveHashPath -Raw).Trim() -split '\s+')[0]
if ($hash -ne $expected) {
    throw "Archive hash mismatch: expected $expected, got $hash"
}

Write-Host "R6_BUNDLE_PATH=$archivePath"
Write-Host "R6_BUNDLE_SHA256=$hash"
Write-Host "R6_TTY_FIX_REPACKAGE_HOST_OK"
