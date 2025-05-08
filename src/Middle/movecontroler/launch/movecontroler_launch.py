from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path=os.path.join(
        get_package_share_directory("movecontroler"),
        "config",
        "movecontroler_params.yaml"
    )

    return LaunchDescription([
        Node(
            package='movecontroler',
            executable='movecontroler',
            name='movecontrolernode',
            namespace="AUH",
            parameters=[config_path],
        ),
    ])