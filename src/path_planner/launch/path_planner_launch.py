from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_interpolated_points',
            default_value='30',
            description='Number of interpolated points on 2d path'
        ),
        DeclareLaunchArgument(
            'point_jump_threshold',
            default_value='0.05',
            description='Threshold of invalid euclidean distance between 3d points in 3d path'
        ),
        Node(
            package='path_planner',
            executable='path_planner',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'num_interpolated_points': LaunchConfiguration('num_interpolated_points'),
                'point_jump_threshold': LaunchConfiguration('point_jump_threshold')
            }]
        )
    ])