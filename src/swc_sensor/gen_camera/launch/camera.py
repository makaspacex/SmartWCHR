import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明并获取启动参数
    return LaunchDescription(
        [
            # 创建节点配置
            Node(
                package="gen_camera",
                executable="camera",
                name="image_publish",
                output="screen",
            ),
        ]
    )
