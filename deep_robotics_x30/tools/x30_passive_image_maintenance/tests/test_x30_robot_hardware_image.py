"""精简 X30 镜像及其内置 HAL 快照的静态合同测试。"""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
TRANSFER_ROOT = PROJECT_ROOT / "docker" / "x30_livox_ros2_transfer"


def read(relative_path: str) -> str:
    return (TRANSFER_ROOT / relative_path).read_text(encoding="utf-8")


# 构建布局检查用于防止意外退回旧镜像。旧镜像会直接占用 Livox/Yesense
# 硬件，或打包尚未完成的平面分割链。
def test_dockerfile_builds_and_installs_x30_only_hal() -> None:
    dockerfile = read("Dockerfile")

    assert (
        "COPY components/robot_hardware_x30/ "
        "/opt/src/robot_hardware_x30/"
    ) in dockerfile
    assert "-DBUILD_X30_ONLY=ON" in dockerfile
    assert "-DBUILD_X30_PROTOCOL_TEST=ON" in dockerfile
    assert "-DBUILD_X30_ROS1_TEST=OFF" in dockerfile
    assert "-DCMAKE_INSTALL_PREFIX=/opt/x30_robot_hardware" in dockerfile
    assert "sha256sum --check SHA256SUMS" in dockerfile
    assert "dpkg-query -W libyaml-cpp-dev" in dockerfile
    assert dockerfile.index("COPY components/robot_hardware_x30/") < (
        dockerfile.index("COPY ws/src/x30_sensor_receiver/")
    )
    assert (
        'CMAKE_PREFIX_PATH="/opt/x30_robot_hardware:${CMAKE_PREFIX_PATH}"'
        in dockerfile
    )


def test_dockerfile_is_passive_receiver_only() -> None:
    dockerfile = read("Dockerfile")

    assert "COPY ws/src/x30_sensor_receiver/" in dockerfile
    assert "--packages-select x30_sensor_receiver" in dockerfile
    assert '"x30_sensor_receiver", "passive_receiver.launch.py"' in dockerfile
    assert "x30_sensor_wire_protocol_test" in dockerfile

    removed_runtime_components = (
        "Livox-SDK2",
        "third_party/plane_seg",
        "livox_ros_driver2",
        "yesense_interface",
        "yesense_std_ros2",
        "x30_livox_tools",
        "x30_plane_seg_core",
        "x30_multi_mid360_ros2",
    )
    for component in removed_runtime_components:
        assert component not in dockerfile


def test_build_script_verifies_installed_hal_without_network() -> None:
    build_script = read("remote/00_build_image.sh")

    assert "verifying installed X30 robot hardware HAL" in build_script
    assert "docker run --rm --network none" in build_script
    assert "/opt/x30_robot_hardware/bin/x30_udp_protocol_test" in build_script
    assert "/opt/x30_robot_hardware/bin/x30_factory_contract_test" in build_script
    assert "librobot_hardware_x30.so" in build_script
    assert "resolved_library" in build_script
    assert 'readlink -f "${expected_library}"' in build_script
    assert 'readlink -f "${resolved_library}"' in build_script


def test_build_script_verifies_only_passive_receiver_and_hal() -> None:
    build_script = read("remote/00_build_image.sh")

    assert "verifying passive 105-to-106 sensor receiver" in build_script
    assert "x30_sensor_receiver_node" in build_script
    assert "passive_receiver.launch.py" in build_script
    assert "Livox SDK" not in build_script
    assert "plane segmentation" not in build_script
    assert "x30_plane_seg_core" not in build_script


# 离线验证阶段必须无法访问实机网络。独立的 zero-only 脚本才是第一个允许
# 使用 host 网络的验证阶段。
def test_offline_script_cannot_reach_robot_network() -> None:
    script = read("remote/30_verify_x30_hal_offline.sh")

    assert "--network none" in script
    assert "x30_udp_protocol_test" in script
    assert "x30_factory_contract_test" in script
    assert "librobot_hardware_x30.so" in script
    assert "resolved_library" in script
    assert 'readlink -f "${expected_library}"' in script
    assert 'readlink -f "${resolved_library}"' in script


def test_zero_script_requires_confirmation_and_safe_config() -> None:
    script = read("remote/31_test_x30_hal_zero.sh")

    assert "CONFIRM_X30_ZERO_ONLY" in script
    assert "configure_non_manual_mode" in script
    assert "configure_navigation_velocity_source" in script
    assert script.index("--network none") < script.index("--network host")
    assert "librobot_hardware_x30.so" in script
    assert "resolved_library" in script
    assert 'readlink -f "${expected_library}"' in script
    assert 'readlink -f "${resolved_library}"' in script
    assert "--network host" in script
    assert "robot_test_x30" in script
    assert "/config/robot_hardware.yaml zero" in script


# 暂存源码清单是镜像可复现性的边界：所有必需接口和协议文件都必须存在，
# 并且必须写入摘要。
def test_staged_hal_manifest_covers_required_contract_files() -> None:
    component = TRANSFER_ROOT / "components/robot_hardware_x30"
    manifest = read("components/robot_hardware_x30/SHA256SUMS")

    required = {
        "CMakeLists.txt",
        "config.yaml",
        "robot_test_x30.cpp",
        "include/robot_hardware_interface.h",
        "include/robot_factory.h",
        "include/deep_robotics/deep_robotics_x30.h",
        "src/deep_robotics/deep_robotics_x30.cpp",
        "tests/x30_factory_contract_test.cpp",
        "tests/x30_udp_protocol_test.cpp",
    }

    for relative_path in required:
        assert (component / relative_path).is_file()
        assert f"  {relative_path}" in manifest


# Release 构建会定义 NDEBUG，因此协议测试必须在包含 <cassert> 前显式恢复
# assert，避免测试断言被编译器移除。
def test_release_protocol_test_keeps_assertions_enabled() -> None:
    protocol_test = read(
        "components/robot_hardware_x30/tests/x30_udp_protocol_test.cpp"
    )

    assert "#ifdef NDEBUG" in protocol_test
    assert "#undef NDEBUG" in protocol_test
    assert protocol_test.index("#undef NDEBUG") < protocol_test.index(
        "#include <cassert>"
    )


# 唯一 SONAME 和相对 RPATH 可防止镜像迁移到另一台机器狗后，把本 X30 库
# 错误解析为宿主机中名称相近的库。
def test_x30_only_hal_uses_unique_soname_and_relative_rpath() -> None:
    cmake = read("components/robot_hardware_x30/CMakeLists.txt")

    assert "PROPERTIES OUTPUT_NAME robot_hardware_x30" in cmake
    assert 'PROPERTIES INSTALL_RPATH "$ORIGIN/../lib"' in cmake
