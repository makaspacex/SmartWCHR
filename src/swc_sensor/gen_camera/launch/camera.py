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
                default_value="20.0",
                description="frequency",
            ),
            Node(
                package="gen_camera",
                executable="camera",
                name="image_publish",
                output="screen",
                parameters=[{'frequency': LaunchConfiguration('frequency')}]
            ),
        ]
    )
