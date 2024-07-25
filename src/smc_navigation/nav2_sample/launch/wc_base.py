import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    urdf_model_path = os.path.join(package_share_dir, "urdf/wheelchair_base.urdf")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    robot_description = ParameterValue(
        Command(["xacro ", str(urdf_model_path)]), value_type=str
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                        "use_gui": False,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {
                        "use_gui": False,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
