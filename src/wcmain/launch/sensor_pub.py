import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    oradar_launch_file = os.path.join(get_package_share_directory("lidar"), "launch", "scan.py")
    imu_launch_file = os.path.join(get_package_share_directory("imu"), "launch", "start.py")

    # description_launch_file = os.path.join(get_package_share_directory("wcmodel"), "launch", "wc_base.py")
    # slam_launch_file = os.path.join(get_package_share_directory("slam_2d"), "launch", "slam_2d.py")
    driver_node = Node(
        package='driver',
        executable='start',
        name='driver'
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(oradar_launch_file)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_launch_file)),
            driver_node
        ]
    )


# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     # # cyq add
#     # imu_to_base_node = Node(
#     #         package='tf2_ros',
#     #         executable='static_transform_publisher',
#     #         name='static_tf_pub',
#     #         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
#     #     )

#     nodes = []

#     ################################## imu nodes ##################################
#     base_link_to_imu_tf_node = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='base_link_to_base_imu',
#         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
#     )

#     rviz_and_imu_node = Node(
#         package='imu',
#         executable='start',
#         name='imu',
#         remappings=[('/wit/imu', '/imu/data')],
#         parameters=[{'port': '/dev/imu_usb'},
#                     {"baud": 9600}],
#         output="screen"

#     )

#     nodes += [base_link_to_imu_tf_node, rviz_and_imu_node]

#     ################################## lidar_nodes ##################################
#     ordlidar_node = Node(
#       package='lidar',
#       executable='scan',
#       name='MS200',
#       output='screen',
#       parameters=[
#         {'device_model': 'MS200'},
#         {'frame_id': 'lidar'},
#         {'scan_topic': '/scan'},
#         {'port_name': '/dev/oradar'},
#         {'baudrate': 230400},
#         {'angle_min': 0.0},
#         {'angle_max': 360.0},
#         {'range_min': 0.05},
#         {'range_max': 20.0},
#         {'clockwise': False},
#         {'motor_speed': 10}
#       ]
#     )

#     # # base_link to laser_frame tf node
#     base_link_to_laser_tf_node = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='base_link_to_base_laser',
#         arguments=['0','0','0.18','0','0','0','base_link','lidar']
#     )

#     # Define LaunchDescription variable
#     # ord = LaunchDescription()
#     # ord.add_action(base_link_to_laser_tf_node)
#     # ord.add_action(ordlidar_node)

#     nodes += [base_link_to_laser_tf_node, ordlidar_node]

#     ################################## driver nodes ##################################

#     driver_node = Node(
#         package='driver',
#         executable='start',
#         name='driver'
#     )

#     nodes += [driver_node]

#     # add nodes to description
#     return LaunchDescription(nodes)
