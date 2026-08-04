from __future__ import annotations

import hashlib
from pathlib import Path


TRANSFER_ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_ROOT = TRANSFER_ROOT / "third_party" / "plane_seg"


def test_plane_seg_upstream_snapshot_hashes() -> None:
    checksum_file = SNAPSHOT_ROOT / "SHA256SUMS"
    assert checksum_file.is_file()

    for line in checksum_file.read_text(encoding="ascii").splitlines():
        expected_hash, relative_path = line.split("  ", maxsplit=1)
        path = SNAPSHOT_ROOT / relative_path
        assert path.is_file(), relative_path
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash


def test_plane_seg_upstream_provenance_is_pinned() -> None:
    provenance = (SNAPSHOT_ROOT / "UPSTREAM_VERSION.md").read_text(
        encoding="utf-8"
    )
    assert "f94dc77c684225eded23f488d5b94baf579fd460" in provenance
    assert "BSD 3-Clause" in provenance
    assert "not treat this unmodified upstream snapshot as X30-compatible" in provenance
