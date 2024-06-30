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

    # 自动补全
    # 需要安装 sudo apt install python3-argcomplete
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/humble/
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    # argcomplete for ros2 & colcon
    eval "$(register-python-argcomplete3 ros2)"
    eval "$(register-python-argcomplete3 colcon)"
}

load_ros2