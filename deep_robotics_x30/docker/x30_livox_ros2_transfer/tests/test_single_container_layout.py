import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_all_sensors_launch_exists_and_starts_three_nodes():
    launch_text = read("ws/src/x30_livox_tools/launch/x30_all_sensors_launch.py")

    assert 'package="livox_ros_driver2"' in launch_text
    assert 'executable="livox_ros_driver2_node"' in launch_text
    assert 'package="x30_livox_tools"' in launch_text
    assert 'executable="time_window_cloud_merger"' in launch_text
    assert 'package="yesense_std_ros2"' in launch_text
    assert 'executable="yesense_node_publisher"' in launch_text
    assert "enable_body_imu" in launch_text
    assert "/x30/body_imu" in launch_text
    assert "/x30/points_merged" in launch_text


def test_single_container_remote_scripts_exist():
    run_text = read("remote/18_run_ros2_all.sh")
    check_text = read("remote/19_check_ros2_all.sh")
    stop_text = read("remote/20_stop_ros2_all.sh")

    assert 'NAME="${NAME:-x30_ros2_sensors}"' in run_text
    assert "--device=" in run_text
    assert "x30_all_sensors_launch.py" in run_text
    assert "x30_livox_ros2 x30_cloud_merger x30_body_imu" in run_text

    assert "/livox/lidar" in check_text
    assert "/livox/imu" in check_text
    assert "/x30/points_merged" in check_text
    assert "/x30/body_imu" in check_text

    assert 'NAME="${NAME:-x30_ros2_sensors}"' in stop_text


def test_docker_default_runs_all_sensors_launch():
    dockerfile_text = read("Dockerfile")
    assert "x30_all_sensors_launch.py" in dockerfile_text


def test_docker_builds_pinned_livox_sdk_with_slave_support():
    dockerfile_text = read("Dockerfile")
    build_script_text = read("remote/00_build_image.sh")
    upstream_text = read("third_party/Livox-SDK2/UPSTREAM_VERSION.md")

    assert "third_party/Livox-SDK2" in dockerfile_text
    assert "livox_lidar_sdk_shared" in dockerfile_text
    assert 'grep -aFq "master_sdk"' in dockerfile_text
    assert 'grep -aFq "master_sdk"' in build_script_text
    assert "v1.3.1" in upstream_text
    assert "f5d9375f84efe2b15bc0a052d3e18482ed13adf4" in upstream_text


def test_livox_slave_mode_is_isolated_from_normal_startup():
    config = json.loads(read("config/x30_multi_mid360_ros2_slave.json"))
    assert config["master_sdk"] is False
    assert "lidar_log_enable" not in config
    host_net_info = config["MID360"]["host_net_info"]
    assert isinstance(host_net_info, list)
    assert len(host_net_info) == 1
    assert host_net_info[0]["multicast_ip"] == "224.1.1.5"
    assert set(host_net_info[0]["lidar_ip"]) == {
        "192.168.2.202",
        "192.168.2.203",
        "192.168.2.204",
        "192.168.2.205",
    }

    probe_text = read("remote/21_probe_livox_slave_mode.sh")
    run_text = read("remote/22_run_ros2_livox_slave.sh")
    stop_text = read("remote/23_stop_ros2_livox_slave.sh")
    check_text = read("remote/24_check_livox_dual_receive.sh")
    diagnose_text = read("remote/25_diagnose_livox_slave.sh")

    assert "Read-only probe" in probe_text
    assert "master_sdk" in probe_text
    assert "lidar_log_enable requires" in probe_text
    assert "must list all four X30 lidars" in probe_text
    assert 'NAME="${NAME:-x30_ros2_livox_slave}"' in run_text
    assert "enable_body_imu:=false" in run_text
    assert "02_factory_livox_off.sh" not in run_text
    assert "06_factory_livox_on.sh" in run_text
    assert 'NAME="${NAME:-x30_ros2_livox_slave}"' in stop_text
    assert "/lidar_points" in check_text
    assert "/livox/lidar" in check_text
    assert "tcpdump" in diagnose_text
    assert "/proc/net/igmp" in diagnose_text
    assert "enable_body_imu:=false" in diagnose_text
    assert "Factory ROS1 remains the Livox master" in diagnose_text
