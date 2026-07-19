from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os


# Replays a .clf dataset through GMapping as a reference implementation,
# publishing the same /map topic and map->odom TF as fastslam2_node so the
# identical RViz session works for both. Run one SLAM node at a time.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'clf_file',
            default_value='../datasets/mit-killian.clf',
            description='Path to .clf dataset file (relative to ros2_ws)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Replay rate in Hz'
        ),
        DeclareLaunchArgument(
            'record_traj',
            default_value='false',
            description='Record the map->base_footprint trajectory to traj_file'
        ),
        DeclareLaunchArgument(
            'traj_file',
            default_value='../results/ros_replay/gmapping.tum',
            description='Output .tum trajectory path (relative to ros2_ws)'
        ),

        # CLF dataset publisher (identical to dataset_slam.launch.py)
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

        # GMapping reference. Filter parameters come from the shared config
        # (also read by the offline gmapping_clf_runner); only launch-specific
        # settings (sim time, frame names) stay inline.
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[
                os.path.join('..', 'configs', 'gmapping', 'default.yaml'),
                {
                    'use_sim_time': True,
                    'base_frame': 'base_footprint',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                },
            ],
        ),

        # Optional trajectory recorder (record_traj:=true). Records gmapping's
        # map->base_footprint trajectory for evo comparison.
        Node(
            package='clf_publisher',
            executable='traj_recorder',
            name='traj_recorder',
            condition=IfCondition(LaunchConfiguration('record_traj')),
            parameters=[{
                'target_frame': 'map',
                'source_frame': 'base_footprint',
                'output_file': LaunchConfiguration('traj_file'),
            }],
            output='screen',
        ),
    ])
