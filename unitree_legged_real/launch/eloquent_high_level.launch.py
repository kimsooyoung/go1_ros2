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

    ros_state_helper = Node(
        package='unitree_legged_real',
        node_executable='ros_state_helper',
        node_name='ros_state_helper',
        output='screen',
        parameters=[{
            "imu_link" : "imu_link",
            "base_link" : "base_link",
            "odom_link" : "odom",
            "publish_tf" : True,
            "verbose" : False,
            "dimension_3d" : False,
        }],
    )

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

    unitree_twist_cmd = Node(
        package='unitree_twist_cmd',
        node_executable='unitree_twist_cmd',
        node_name='unitree_twist_cmd',
        output='screen',
        parameters=[{
            "default_mode" : 1,
            "verbose" : True,
        }],
    )

    # TODO
    static_transform_publisher = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'base_link']
    )

    return launch.LaunchDescription([
        ros2_udp,
        ros_state_helper,
        # unitree_joy_cmd,
        unitree_twist_cmd,
        static_transform_publisher,
    ])
