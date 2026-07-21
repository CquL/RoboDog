from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    publish_freq = LaunchConfiguration("publish_freq")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    cloud_input_topic = LaunchConfiguration("cloud_input_topic")
    cloud_output_topic = LaunchConfiguration("cloud_output_topic")
    cloud_output_frame = LaunchConfiguration("cloud_output_frame")
    cloud_window_ms = LaunchConfiguration("cloud_window_ms")
    cloud_min_clouds = LaunchConfiguration("cloud_min_clouds")

    enable_body_imu = LaunchConfiguration("enable_body_imu")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_topic_ros = LaunchConfiguration("imu_topic_ros")
    imu_topic_raw = LaunchConfiguration("imu_topic_raw")

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="x30_livox_ros2",
        output="screen",
        parameters=[
            {"xfer_format": ParameterValue(xfer_format, value_type=int)},
            {"multi_topic": ParameterValue(multi_topic, value_type=int)},
            {"data_src": 0},
            {"publish_freq": ParameterValue(publish_freq, value_type=float)},
            {"output_data_type": 0},
            {"frame_id": lidar_frame_id},
            {"user_config_path": config_path},
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
    )

    cloud_merger = Node(
        package="x30_livox_tools",
        executable="time_window_cloud_merger",
        name="time_window_cloud_merger",
        output="screen",
        parameters=[
            {"input_topic": cloud_input_topic},
            {"output_topic": cloud_output_topic},
            {"output_frame": cloud_output_frame},
            {"window_ms": ParameterValue(cloud_window_ms, value_type=float)},
            {"min_clouds": ParameterValue(cloud_min_clouds, value_type=int)},
        ],
    )

    body_imu = Node(
        package="yesense_std_ros2",
        executable="yesense_node_publisher",
        name="x30_body_imu",
        output="screen",
        condition=IfCondition(enable_body_imu),
        parameters=[
            {"serial_port": serial_port},
            {"baud_rate": ParameterValue(baud_rate, value_type=int)},
            {"frame_id": imu_frame_id},
            {"driver_type": "linux_serial"},
            {"imu_topic_ros": imu_topic_ros},
            {"imu_topic": imu_topic_raw},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value="/config/x30_multi_mid360_ros2.json",
                description="Path to X30 multi-MID360 Livox config.",
            ),
            DeclareLaunchArgument(
                "xfer_format",
                default_value="0",
                description="0=PointCloud2, 1=Livox CustomMsg.",
            ),
            DeclareLaunchArgument(
                "multi_topic",
                default_value="0",
                description="0=merged Livox topic, 1=one topic per lidar.",
            ),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("lidar_frame_id", default_value="lidar_link"),
            DeclareLaunchArgument("cloud_input_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("cloud_output_topic", default_value="/x30/points_merged"),
            DeclareLaunchArgument("cloud_output_frame", default_value="lidar_link"),
            DeclareLaunchArgument("cloud_window_ms", default_value="100.0"),
            DeclareLaunchArgument("cloud_min_clouds", default_value="1"),
            DeclareLaunchArgument(
                "enable_body_imu",
                default_value="true",
                description="Start the X30 body Yesense IMU node in this container.",
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_topic_ros", default_value="/x30/body_imu"),
            DeclareLaunchArgument("imu_topic_raw", default_value="/x30/body_imu_raw"),
            livox_driver,
            cloud_merger,
            body_imu,
        ]
    )
