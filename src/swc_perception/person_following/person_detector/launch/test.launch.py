# person_detector/launch/person_detector_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

# person_tracker_node = person_detector.person_tracker_node:main
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='person_detector',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
            parameters=[{'param_name': 'value'}],  # 可选的参数
            remappings=[('/old/topic', '/new/topic')]  # 可选的主题重映射
        ),
        Node(
            package='person_detector',
            executable='persons_visualize_node',
            name='persons_visualize_node',
            output='screen',
            parameters=[{'param_name': 'value'}],  # 可选的参数
            remappings=[('/old/topic', '/new/topic')]  # 可选的主题重映射
        ),
        Node(
            package='video_pub',
            executable='video_pub_node',
            name='video_publisher_node',
            output='screen',
            parameters=[{'param_name': 'value'}],  # 可选的参数
            remappings=[('/old/topic', '/new/topic')]  # 可选的主题重映射
        ),        
    ])
