from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('marker_file', default_value=os.path.expanduser('~/markers.txt')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower',
            parameters=[{'use_sim_time': sim}],
        ),
        Node(
            package='wall_follower',
            executable='see_marker.py',
            name='see_marker',
            parameters=[{'use_sim_time': sim}],
        ),
        Node(
            package='wall_follower',
            executable='point_transformer.py',
            name='point_transformer',
            parameters=[{'use_sim_time': sim, 'marker_file': LaunchConfiguration('marker_file')}],
        ),
    ])
