#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker_node = Node(
        package='lab1_pkg',
        executable='talker.py',   # use this line if you want to try py node
        # executable='talker_cpp',   # use this line if you want to try cpp node
        name='talker',
        parameters=[
            {'v': 10.0},  # Adjust these values as needed
            {'d': 3.0}
        ],
        output='screen',
    )

    relay_node = Node(
        package='lab1_pkg',
        executable='relay.py',   # use this line if you want to try py node
        # executable='relay_cpp',   # use this line if you want to try cpp node
        name='relay',
        output='screen',
    )

    return LaunchDescription([
        talker_node,
        relay_node
    ])