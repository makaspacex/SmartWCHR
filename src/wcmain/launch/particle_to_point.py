from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="wcmain",
                executable="particle_to_point",
                name="particle_to_point",
                output="screen",
            )
        ]
    )
