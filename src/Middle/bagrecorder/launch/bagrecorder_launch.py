from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bagrecorder_node = Node(
        name="bagrecordernode",
        namespace="AUH",
        package="bagrecorder",
        executable="bagrecorder_exe",
    )
    return LaunchDescription([bagrecorder_node])