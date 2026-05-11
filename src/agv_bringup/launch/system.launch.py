from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/root/agv_ws/src/agv_bringup/launch/localization.launch.py'
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/root/agv_ws/src/agv_bringup/launch/navigation.launch.py'
        )
    )

    return LaunchDescription([
        localization,
        navigation
    ])

