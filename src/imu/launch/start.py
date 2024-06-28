from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # # cyq add
    # imu_to_base_node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_tf_pub',
    #         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    #     )

    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_stf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    ) 

    rviz_and_imu_node = Node(
        package='imu',
        executable='start',
        name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/ttyUSB1'},
                    {"baud": 9600}],
        output="screen"

    )

    return LaunchDescription(
        [
            base_link_to_imu_tf_node,
            rviz_and_imu_node
          
        ]
    )