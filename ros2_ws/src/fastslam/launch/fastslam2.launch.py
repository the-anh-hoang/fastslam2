from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='fastslam',
            executable='fastslam2_node',
            name='fastslam2_node',
            parameters=[os.path.join(
                get_package_share_directory('fastslam'),
                'config', 'fastslam_params.yaml'
            )]
        )
    ])