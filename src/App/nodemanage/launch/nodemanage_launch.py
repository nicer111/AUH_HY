from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path=os.path.join(
        get_package_share_directory("nodemanage"),
        "config",
        "nodemanage_params.yaml"
    )

    return LaunchDescription([
        Node(
            package='nodemanage',
            executable='nodemanage',
            name='nodemanagenode',
            namespace="AUH",
            parameters=[config_path],
        ),
    ])