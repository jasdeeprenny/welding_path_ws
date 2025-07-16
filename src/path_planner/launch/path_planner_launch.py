from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('path_planner'), 'config'
    )
    default_config = os.path.join(config_dir, 'path_planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Full path to YAML config file for the node'
        ),
        Node(
            package='path_planner',
            executable='path_planner',
            name='path_planner_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])