import launch_ros
import launch 

def generate_launch_description():
    driver_node = launch_ros.actions.Node(
	         package='driver',
	         executable='start',
	         name='driver_control',
	         output='screen',
	    )
    odom_msg_node = launch_ros.actions.Node(
	         package='driver',
	         executable='odom',
	         name='odom_msg',
	         output='screen',
	    )
    return launch.LaunchDescription([
	driver_node,
	odom_msg_node,
	])
