"""启动 105-to-106 被动接收端，不占用传感器硬件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 网络参数定义 105-to-106 信任边界，Topic 参数定义容器内算法使用的
    # ROS2 接口。
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "allowed_source_ip", default_value="192.168.1.105"
            ),
            DeclareLaunchArgument("point_cloud_port", default_value="56110"),
            DeclareLaunchArgument("imu_port", default_value="56111"),
            DeclareLaunchArgument("odometry_port", default_value="56112"),
            DeclareLaunchArgument(
                "point_cloud_topic", default_value="/x30/lidar_points"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/x30/body_imu"),
            DeclareLaunchArgument(
                "odometry_topic", default_value="/x30/leg_odom"
            ),
            DeclareLaunchArgument(
                "max_payload_bytes", default_value="67108864"
            ),
            # ParameterValue 保证数值 launch 参数保持整数类型；
            # 直接使用 LaunchConfiguration 时会以文本形式传入。
            Node(
                package="x30_sensor_receiver",
                executable="x30_sensor_receiver_node",
                name="x30_sensor_receiver",
                output="screen",
                parameters=[
                    {"bind_address": LaunchConfiguration("bind_address")},
                    {
                        "allowed_source_ip": LaunchConfiguration(
                            "allowed_source_ip"
                        )
                    },
                    {
                        "point_cloud_port": ParameterValue(
                            LaunchConfiguration("point_cloud_port"),
                            value_type=int,
                        )
                    },
                    {
                        "imu_port": ParameterValue(
                            LaunchConfiguration("imu_port"), value_type=int
                        )
                    },
                    {
                        "odometry_port": ParameterValue(
                            LaunchConfiguration("odometry_port"),
                            value_type=int,
                        )
                    },
                    {
                        "point_cloud_topic": LaunchConfiguration(
                            "point_cloud_topic"
                        )
                    },
                    {"imu_topic": LaunchConfiguration("imu_topic")},
                    {
                        "odometry_topic": LaunchConfiguration(
                            "odometry_topic"
                        )
                    },
                    {
                        "max_payload_bytes": ParameterValue(
                            LaunchConfiguration("max_payload_bytes"),
                            value_type=int,
                        )
                    },
                ],
            ),
        ]
    )
