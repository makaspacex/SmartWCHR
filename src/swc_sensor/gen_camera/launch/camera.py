import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明并获取启动参数
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frequency",
                default_value="8.0",
                description="frequency",
            ),
            DeclareLaunchArgument(
                "showtime",
                default_value="false",
                description="if showtime on frame image",
            ),
            Node(
                package="gen_camera",
                executable="camera",
                name="camera_image_publish",
                output="screen",
                parameters=[{'frequency': LaunchConfiguration('frequency'),'showtime': LaunchConfiguration('showtime')}]
            ),
        ]
    )
