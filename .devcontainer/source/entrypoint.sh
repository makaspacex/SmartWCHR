#!/bin/bash

# 获取设置的用户id，默认为1000
USER_ID=${CUID:-1000}
UNAME=user

# 授予用户免密码root权限
echo "$UNAME ALL=(ALL) NOPASSWD: NOPASSWD: ALL"  > /etc/sudoers.d/sudonopasswd

# 创建和主机用户相同uid的用户，名为user
useradd --shell /bin/zsh -u $USER_ID -o -c "" -m $UNAME
usermod -aG root $UNAME

# 设置目录
export HOME=/home/$UNAME
rm -rf $HOME && cp -r /root $HOME && chown -R $UNAME:$UNAME $HOME

exec /usr/local/bin/gosu user "$@"
