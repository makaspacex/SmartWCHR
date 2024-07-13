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

    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/imu_usb'},
                    {"baud": 9600}],
        output="screen"
    )

    return LaunchDescription(
        [
            imu_node
        ]
    )