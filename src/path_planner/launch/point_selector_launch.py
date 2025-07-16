from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_clicked_points',
            default_value='2',
            description='Number of points to click'
        ),

        Node(
            package='point_selector_gui',
            executable='point_selector',
            name='point_selector_node',
            output='screen',
            parameters=[{
                'num_clicked_points': LaunchConfiguration('num_clicked_points')
            }]
        )
    ])