from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'clf_file',
            default_value='/root/ros2_ws/datasets/fr079.clf',
            description='Path to .clf dataset file'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Replay rate in Hz'
        ),

        # CLF dataset publisher
        Node(
            package='clf_publisher',
            executable='clf_node',
            name='clf_publisher',
            parameters=[{
                'clf_file': LaunchConfiguration('clf_file'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'laser_x_offset': 0.0,  # Intel dataset: laser at robot center
                'laser_y_offset': 0.0,
            }],
            output='screen',
        ),

        # FastSLAM2 node
        Node(
            package='fastslam',
            executable='fastslam2_node',
            name='fastslam2_node',
            parameters=[
                os.path.join('/root/ros2_ws/src/fastslam/config', 'fastslam_params.yaml')
            ],
            output='screen',
        ),
    ])
