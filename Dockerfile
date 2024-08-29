FROM makaspacex/smartwchr:base

ARG USERNAME=user
ARG UID=1000
ARG GID=$UID

ENV ROS_DISTRO=humble

# ultralytics
RUN pip3 config set global.index-url https://mirrors.bfsu.edu.cn/pypi/web/simple && \
    pip3 install ultralytics

# torchreid
RUN pip3  install tb-nightly -i https://mirrors.aliyun.com/pypi/simple && \
    git clone https://mirror.ghproxy.com/https://github.com/KaiyangZhou/deep-person-reid.git && \
    cd deep-person-reid/ && pip3 install -r requirements.txt && \
    python3 setup.py develop && cd .. && rm -rf deep-person-reid

# update
COPY scripts/ros_shell.sh /
RUN echo "" >> ~/.zshrc && echo '[[ -f /ros_shell.sh ]] && source /ros_shell.sh' >> ~/.zshrc

RUN apt update && \
    apt install -y --no-install-recommends \
    udev && \
    apt clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY scripts/rules/ /etc/udev/rules.d/

# Create workspace so that user own this directory
WORKDIR /ros2_ws

# set entrypoint
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod a+x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["zsh"]
