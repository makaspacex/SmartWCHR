function load_ws_ros2() {
    if [[ -f install/local_setup.zsh ]]; then
        . install/local_setup.zsh
    fi
}

function load_ros2() {
    # 初始化全局的ros2
    if [[ -f /opt/ros/"$ROS_DISTRO"/setup.zsh ]]; then
        . /opt/ros/"$ROS_DISTRO"/setup.zsh
    fi
    load_ws_ros2
}
load_ros2