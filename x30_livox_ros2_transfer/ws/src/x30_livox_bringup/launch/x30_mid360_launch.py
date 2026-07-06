from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    publish_freq = LaunchConfiguration("publish_freq")
    frame_id = LaunchConfiguration("frame_id")

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
            {"frame_id": frame_id},
            {"user_config_path": config_path},
            {"cmdline_input_bd_code": "livox0000000001"},
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
                description="0=merged topic, 1=one topic per lidar.",
            ),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("frame_id", default_value="lidar_link"),
            livox_driver,
        ]
    )
