from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    lora_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("lora"),
                "launch",
                "lora_launch.py"
            )
        )
    )
    va500p_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("va500p"),
                "launch",
                "va500p_launch.py"
            )
        )
    )
    light_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("light"),
                "launch",
                "light_launch.py"
            )
        )
    )
    battery_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("battery"),
                "launch",
                "battery_launch.py"
            )
        )
    )
    usbcam_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("usb_cam"),
                "launch",
                "camera.launch.py"
            )
        )
    )
    nodemanage_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("nodemanage"),
                "launch",
                "nodemanage_launch.py"
            )
        )
    )
    bagrecorder_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("bagrecorder"),
                "launch",
                "bagrecorder_launch.py"
            )
        )
    )
    oculus_launch = IncludeLaunchDescription(
        launch_description_source= PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("oculus_ros2"),
                "launch",
                "default.launch.py"
            )
        )
    )
    navi_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('navi'),
                'launch',
                'navi_launch.py'
            )
        )
    )
    tfbroadcaster_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tfbroadcaster'),
                'launch',
                'tfbroadcaster_launch.py'
            )
        )
    )
    controlmode_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('controlmode'),
                'launch',
                'controlmode_launch.py'
            )
        )
    )
    foxglove_bridge_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )
    return LaunchDescription([lora_launch,va500p_launch,light_launch,battery_launch,usbcam_launch,foxglove_bridge_launch,oculus_launch,nodemanage_launch,bagrecorder_launch,navi_launch,tfbroadcaster_launch,controlmode_launch])
