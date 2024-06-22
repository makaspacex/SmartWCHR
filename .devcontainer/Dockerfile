FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# mirror repo by bfsu
RUN echo deb https://mirrors.bfsu.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse > /etc/apt/sources.list \
    && echo deb https://mirrors.bfsu.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse >> /etc/apt/sources.list \
    && echo deb https://mirrors.bfsu.edu.cn/ubuntu-ports/ jammy-backports main restricted universe multiverse >> /etc/apt/sources.list \
    && echo deb https://mirrors.bfsu.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse >> /etc/apt/sources.list

RUN apt update -q
RUN apt install -y curl gnupg2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.bfsu.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
RUN apt update -q \
    && apt upgrade -q -y
# Install some dependencies packages
RUN apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
