import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from lifecycle_msgs.msg import Transition
import launch
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import StateTransition
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
import launch_ros
from launch.actions import LogInfo


def generate_launch_description():

    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    declare_map_file_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_share_dir, "map/YIFU01.yaml"),
        description="map file path",
    )
    nav2_map_file = LaunchConfiguration("map")
    # map_server_args = {
    #     "frame_id": "map",
    #     "topic_name": "map",
    #     "use_sim_time":use_sim_time,
    #     "yaml_filename": nav2_map_file,
    # }
    # start_map_server_cmd = Node(
    #     package="nav2_map_server",
    #     executable="map_server",
    #     name="map_server",
    #     parameters=[map_server_args],
    #     output="screen",
    # )

    lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_for_show",
        namespace="",
        output="screen",
        parameters=[{"yaml_filename": nav2_map_file}],
    )
    from launch import LaunchDescription
    from launch.actions import RegisterEventHandler, TimerAction
    from launch_ros.actions import Node, LifecycleNode
    from launch_ros.event_handlers import OnStateTransition
    from launch_ros.events.lifecycle import ChangeState
    from lifecycle_msgs.msg import Transition

    configure_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(lifecycle_node),
                            transition_id=Transition.TRANSITION_CONFIGURE
                        ),
                    ],
                ),
            ],
        )
    )
    
    # 监听启动时的自动配置请求
    auto_configure = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state="configuring",
            entities=[
                LogInfo(msg="Lifecycle node '/map_server' is configuring..."),
                # ChangeState(
                # lifecycle_node_matcher=launch.events.matches_action(lifecycle_node),
                # transition_id=Transition.TRANSITION_CONFIGURE
                # ),
                # LogInfo(msg="Lifecycle node '/map_server' has been configured."),
                # ChangeState(
                # lifecycle_node_matcher=launch.events.matches_action(lifecycle_node),
                # transition_id=Transition.TRANSITION_ACTIVATE
                # ),
                # LogInfo(msg="Lifecycle node '/map_server' has been activated.")
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(lifecycle_node)
    ld.add_action(auto_configure)

    return ld
