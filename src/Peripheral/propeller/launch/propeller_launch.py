from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path=os.path.join(
        get_package_share_directory("propeller"),
        "config",
        "propeller_params.yaml"
    )

    return LaunchDescription([
        Node(
            package='propeller',
            executable='propeller',
            name='propellernode',
            namespace="AUH",
            parameters=[config_path],
        ),
    ])