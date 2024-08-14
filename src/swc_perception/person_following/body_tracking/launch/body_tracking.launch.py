import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='body_tracking',        # 包名
            executable='body_tracking',     # 可执行文件名
            # name='body_tracking_node',      # 不指定节点名称，因为这个可执行文件会同时启动多个节点
            output='screen',                # 输出到屏幕
            # parameters=[{'param_name': 'param_value'}],  # 节点参数（如果有的话）
            # remappings=[('/old/topic', '/new/topic')]    # 主题重映射（如果有的话）
        ),
        launch_ros.actions.Node(
            package='person_detector',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
        ),
    ])
