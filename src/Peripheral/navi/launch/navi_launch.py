import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    config_path=os.path.join(
        get_package_share_directory("navi"),
        "config",
        "navi_params.yaml"
    )

    return LaunchDescription([
        Node(
            package='navi',
            executable='navi',
            name='navinode',
            namespace="AUH",
            parameters=[config_path],
        ),
    ])

