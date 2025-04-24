from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    light_node = Node(
        name="lightnode",
        namespace="AUH",
        package="light",
        executable="light",
    )
    return LaunchDescription([light_node])