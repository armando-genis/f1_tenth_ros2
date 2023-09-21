from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='obstacle_pkg',
            executable='Frenet_node',
            name='Frenet_node',
            output='screen',
        )
    ])
