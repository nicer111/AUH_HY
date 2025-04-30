#!/bin/bash
source /opt/ros/humble/setup.bash  # 加载ROS2环境
source ~/clionRemote/ros_ws/install/local_setup.bash  # 如果有工作空间，请加载工作空间环境
exec "$@"  # 执行传入的命令
