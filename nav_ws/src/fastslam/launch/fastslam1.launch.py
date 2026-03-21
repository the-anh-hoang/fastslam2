from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='fastslam',
            executable='fastslam_node',
            name='fastslam_node',
            parameters=[os.path.join(
                get_package_share_directory('fastslam'),
                'config', 'fastslam_params.yaml'
            )]
            # abs path: /home/harry/dev/nav_isaacsim/nav_ws/src/fastslam/config/fastslam_params.yaml
        )
    ])