from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    output_frame = LaunchConfiguration("output_frame")
    window_ms = LaunchConfiguration("window_ms")
    min_clouds = LaunchConfiguration("min_clouds")

    merger = Node(
        package="x30_livox_tools",
        executable="time_window_cloud_merger",
        name="x30_cloud_merger",
        output="screen",
        parameters=[
            {"input_topic": input_topic},
            {"output_topic": output_topic},
            {"output_frame": output_frame},
            {"window_ms": ParameterValue(window_ms, value_type=float)},
            {"min_clouds": ParameterValue(min_clouds, value_type=int)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("output_topic", default_value="/x30/points_merged"),
            DeclareLaunchArgument("output_frame", default_value="lidar_link"),
            DeclareLaunchArgument("window_ms", default_value="100.0"),
            DeclareLaunchArgument("min_clouds", default_value="1"),
            merger,
        ]
    )
