import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    urdf_model_path = os.path.join(package_share_dir, "urdf/wheelchair_base.urdf")
    urdf_model_path = os.path.join(package_share_dir, "urdf/gkchair01_base.urdf")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    robot_description = ParameterValue(
        Command(["xacro ", str(urdf_model_path)]), value_type=str
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="robot_name, gk01/v1",
            ),
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
