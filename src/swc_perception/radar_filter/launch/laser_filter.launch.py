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
                default_value="180",
                description="Start angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "end_angle",
                default_value="70",
                description="End angle for the laser filter",
            ),
            
            DeclareLaunchArgument(
                "sub_topic",
                default_value="scan_s2_raw",
                description="subscription topic",
            ),
            DeclareLaunchArgument(
                "pub_topic",
                default_value="scan_s2_filter",
                description="publisher topic",
            ),
            # 创建节点配置
            Node(
                package="radar_filter",
                executable="laser_filter",
                name="laser_filter",
                parameters=[{
                    "start_angle": LaunchConfiguration("start_angle"),
                    "end_angle": LaunchConfiguration("end_angle"),
                    "sub_topic": LaunchConfiguration("sub_topic"),
                    "pub_topic": LaunchConfiguration("pub_topic"),
                    }],
                output="screen",
            ),
        ]
    )
