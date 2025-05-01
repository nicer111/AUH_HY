#!/bin/bash
source /opt/ros/rolling/setup.bash  # 加载ROS2环境
source /root/WorkSpace/AUH_WS/install/local_setup.bash  # 如果有工作空间，请加载工作空间环境
exec "$@"  # 执行传入的命令
