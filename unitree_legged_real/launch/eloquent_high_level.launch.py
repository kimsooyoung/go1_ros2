import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    level = "highlevel"

    ros2_udp = Node(
        package='unitree_legged_real',
        node_executable='ros2_udp',
        node_name='node_ros2_udp',
        output='screen',
        arguments=[level],
    )

    return launch.LaunchDescription([
        ros2_udp,
    ])