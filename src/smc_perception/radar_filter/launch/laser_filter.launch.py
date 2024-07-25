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
                "start_angle",
                default_value="190",
                description="Start angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "end_angle",
                default_value="70",
                description="End angle for the laser filter",
            ),
            # 创建节点配置
            Node(
                package="radar_filter",
                executable="laser_filter",
                name="laser_filter",
                parameters=[
                    {"start_angle": LaunchConfiguration("start_angle")},
                    {"end_angle": LaunchConfiguration("end_angle")},
                ],
                output="screen",
            ),
        ]
    )
