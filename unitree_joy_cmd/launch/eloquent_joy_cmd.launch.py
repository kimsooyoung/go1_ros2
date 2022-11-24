import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    unitree_joy_cmd = Node(
        package='unitree_joy_cmd',
        node_executable='unitree_joy_cmd',
        node_name='unitree_joy_cmd',
        output='screen',
        parameters=[{
            "roll_gain" : -0.8,
            "pitch_gain" : 0.5,
            "yaw_gain" : 0.3,
            "bodyheight_gain" : 0.2,
            "speed_gain" : 1.0,
        }],
    )

    return launch.LaunchDescription([
        unitree_joy_cmd,
    ])