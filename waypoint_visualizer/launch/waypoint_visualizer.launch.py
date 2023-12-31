from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    waypoints_path = os.path.join(get_package_share_directory('regulated_pure_pursuit'),'config','waypoints.yaml')
    
    
    return LaunchDescription([
        Node(
            package='waypoint_visualizer',
            executable='waypoint_visualizer_node',
            name='waypoint_visualizer_node',
            output='screen',
            parameters=[waypoints_path]
        )
    ])
