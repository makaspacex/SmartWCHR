import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(Path(package_share_dir) / Path("config/slam2d.rviz"))],
                output="screen",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                arguments=[
                    "-configuration_directory",
                    os.path.join(package_share_dir, "config"),
                    "-configuration_basename",
                    "slam2d.lua",
                ],
                remappings=[
                    ("scan", "scan"),
                    ("imu", "imu/data_raw"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                parameters=[{"use_sim_time": use_sim_time}, {"resolution": 0.05}],
            ),
        ]
    )
