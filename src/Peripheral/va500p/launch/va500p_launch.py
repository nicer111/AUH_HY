from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config_path=os.path.join(
        get_package_share_directory("va500p"),
        "config",
        "usart_params.yaml"
    )

    return LaunchDescription([
        Node(
            package='va500p',
            executable='va500p',
            name='va500pnode',
            namespace="AUH",
            parameters=[config_path],
        ),
    ])