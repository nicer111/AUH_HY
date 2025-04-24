# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

# 确保 config 目录被正确加入 sys.path
sys.path.append(os.path.dirname(__file__))
# 导入 camera_config
try:
    from camera_config import CameraConfig, USB_CAM_DIR
except ImportError as e:
    print(f"Error importing camera_config: {e}")
    sys.exit(1)

# 配置摄像头
CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera1',
        namespace='AUH',
        param_path=Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    )
)

#CAMERAS.append(
#    CameraConfig(
#        name='camera2',
#        param_path=Path(USB_CAM_DIR, 'config', 'params_2.yaml')
#    )
#)

def generate_launch_description():
    ld = LaunchDescription()

    # 参数设置
    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str, help='Name for device', default='usb_cam')

    # 创建 camera 节点
    camera_nodes = [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
        for camera in CAMERAS
    ]

    # 分组所有的 camera 节点
    camera_group = GroupAction(camera_nodes)

    # 将组添加到 LaunchDescription 中
    ld.add_action(camera_group)
    return ld