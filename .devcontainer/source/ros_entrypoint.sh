#!/bin/zsh

set -e
# setup ros2 environment
# source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [[ -f install/local_setup.zsh ]]; then
    source install/local_setup.zsh --
fi

exec "$@"