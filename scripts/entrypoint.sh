#!/bin/bash
set -e
# 获取设置的用户id，默认为1000
USER_ID=${CUID:-1000}
UNAME=user

# 保存当前目录，一般是工作目录
PWDD=$(pwd)

# 授予用户免密码root权限
echo "$UNAME ALL=(ALL) NOPASSWD: NOPASSWD: ALL"  > /etc/sudoers.d/sudonopasswd

# 创建和主机用户相同uid的用户，名为user
useradd --shell /bin/zsh -u $USER_ID -o -c "" -m $UNAME
usermod -aG root $UNAME
usermod -aG dialout $UNAME

# 设置目录与SHELL环境变量(重要！)
export HOME=/home/$UNAME
export SHELL=/bin/zsh

# 初始化home目录
cd /root && cp -rf $(ls -A1 --color=never) $HOME && chown -R $UNAME:$UNAME $HOME

# 将工作目录的权限改为当前用户
cd $PWDD && chown $UNAME:$UNAME $PWDD

# 执行用户自定义程序
exec /usr/local/bin/gosu $UNAME "$@"
