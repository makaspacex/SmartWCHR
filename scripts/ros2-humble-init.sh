#!/bin/env bash

set -e

export ROS_DISTRO=humble

# change time to china
echo 'Asia/Shanghai' > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

# install certificates to avoid ssl warning
apt install -y --no-install-recommends ca-certificates

# change repo to china
echo "deb https://mirrors.bfsu.edu.cn/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list
echo "deb https://mirrors.bfsu.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb https://mirrors.bfsu.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb https://mirrors.bfsu.edu.cn/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list

# update repo catch and upgrade system packages to latest
apt update && apt upgrade

# install basic tools
apt install -y --no-install-recommends file dirmngr gpg curl gnupg2 htop usbutils zsh tzdata vim tmux git python3-pip xauth  net-tools udev

# change default shell to zsh
chsh -s /bin/zsh

# Install ros-humble-ros-core
curl -sSL https://mirror.ghproxy.com/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.bfsu.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
apt update && \
apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-core \
    ros-$ROS_DISTRO-demo-nodes-cpp ros-$ROS_DISTRO-demo-nodes-py \
    build-essential git python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool


# install livox sdk2
git clone https://mirror.ghproxy.com/https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2/ && mkdir build && cd build && \
    cmake .. && make -j && make install


apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros ros-$ROS_DISTRO-slam-toolbox \
    python3-argcomplete ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-robot-localization

apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop-full


apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# lsleida
apt install -y --no-install-recommends \
    libpcap-dev libboost1.74-dev \
    ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib ros-$ROS_DISTRO-pcl-conversions

# navigation2
apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# rtabmap-ros
apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-rtabmap-ros

# torch
pip3 config set global.index-url https://mirrors.bfsu.edu.cn/pypi/web/simple && \
    pip3 install h5py==3.10 && \
    pip3 install pyserial numpy rich minimalmodbus transforms3d apscheduler cv_bridge scikit-learn && \
    pip3 install torch torchvision torchaudio

# install oh-my-zsh and tools
sh -c "$(curl -fsSL https://mirror.ghproxy.com/https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone https://mirror.ghproxy.com/https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    sed -i "s/plugins=(git.*)$/plugins=(git zsh-autosuggestions)/" ~/.zshrc

# ultralytics
pip3 config set global.index-url https://mirrors.bfsu.edu.cn/pypi/web/simple && \
    pip3 install ultralytics

# torchreid
pip3  install tb-nightly -i https://mirrors.aliyun.com/pypi/simple && \
    git clone https://mirror.ghproxy.com/https://github.com/KaiyangZhou/deep-person-reid.git && \
    cd deep-person-reid/ && pip3 install -r requirements.txt && \
    python3 setup.py develop && cd .. && rm -rf deep-person-reid

