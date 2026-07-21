from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


TRANSFER_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = TRANSFER_ROOT / "ws" / "src" / "x30_plane_seg_core"


def test_core_package_is_offline_only() -> None:
    required = [
        PACKAGE_ROOT / "CMakeLists.txt",
        PACKAGE_ROOT / "package.xml",
        PACKAGE_ROOT / "include/x30_plane_seg_core/grid_map_adapter.hpp",
        PACKAGE_ROOT / "include/x30_plane_seg_core/plane_seg_core.hpp",
        PACKAGE_ROOT / "src/grid_map_adapter.cpp",
        PACKAGE_ROOT / "src/plane_seg_core.cpp",
        PACKAGE_ROOT / "src/factory_block_fitter.cpp",
        PACKAGE_ROOT / "src/factory_incremental_plane_estimator.cpp",
        PACKAGE_ROOT / "src/factory_plane_segmenter.cpp",
    ]
    for path in required:
        assert path.is_file(), path

    package_xml = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {element.text for element in package_xml if element.text}
    assert "rclcpp" not in dependencies
    assert not list(PACKAGE_ROOT.rglob("*.launch.py"))

    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in PACKAGE_ROOT.rglob("*")
        if path.suffix in {".cpp", ".hpp"}
    )
    for forbidden in (
        "192.168.1.103",
        "49999",
        "sendto(",
        "socket(",
        "rclcpp::",
    ):
        assert forbidden not in source


def test_recovered_factory_core_constants_are_explicit() -> None:
    block_fitter = (
        PACKAGE_ROOT / "src/factory_block_fitter.cpp"
    ).read_text(encoding="utf-8")
    estimator = (
        PACKAGE_ROOT / "src/factory_incremental_plane_estimator.cpp"
    ).read_text(encoding="utf-8")

    assert "segmenter.setMaxError(0.025F)" in block_fitter
    assert "segmenter.setMaxAngle(15.0F)" in block_fitter
    assert "segmenter.setMaxAngleToFloor(15.0F)" in block_fitter
    assert "segmenter.setSearchRadius(0.08F)" in block_fitter
    assert "segmenter.setMinPoints(mMinPoint)" in block_fitter
    assert "block.type = 0" in block_fitter

    assert "(void)iNormal" in estimator
    assert "(void)iMaxAngle" in estimator
    assert "std::abs(plane[2]) < std::cos(iMaxAngleToFloor)" in estimator

    # The recovered X30 binary stores this API flag but never reads it in go().
    assert block_fitter.count("mComputeVerticalPlane") == 1


def test_docker_build_verifies_pinned_baseline_and_core_contract() -> None:
    dockerfile = (TRANSFER_ROOT / "Dockerfile").read_text(encoding="utf-8")
    build_script = (
        TRANSFER_ROOT / "remote/00_build_image.sh"
    ).read_text(encoding="utf-8")

    assert "COPY third_party/plane_seg/ /opt/src/x30-plane-seg-upstream/" in dockerfile
    assert "sha256sum --check SHA256SUMS" in dockerfile
    assert "ros2 pkg prefix x30_plane_seg_core" in build_script
    assert "ctest --test-dir /ws/build/x30_plane_seg_core" in build_script
    assert '"^x30_plane_seg_core_.*_test$"' in build_script
