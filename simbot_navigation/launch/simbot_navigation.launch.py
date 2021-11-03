#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    simbot_node_node=Node(
        package='simbot_navigation',
        executable='simbot_node',
        output='screen',
        name='simbot_node'
    )

    return LaunchDescription([
        simbot_node_node
    ])