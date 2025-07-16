from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    path_planner_dir = get_package_share_directory('path_planner')

    # Paths to launch files
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    point_selector_launch = os.path.join(path_planner_dir, 'launch', 'point_selector_launch.py')
    path_planner_launch = os.path.join(path_planner_dir, 'launch', 'path_planner_launch.py')

    # Paths to config files
    realsense_config = os.path.join(path_planner_dir, 'config', 'realsense_config.yaml')
    point_selector_config = os.path.join(path_planner_dir, 'config', 'point_selector_params.yaml')
    path_planner_config = os.path.join(path_planner_dir, 'config', 'path_planner_params.yaml')


    return LaunchDescription([
        # Launch RealSense node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'config_file': realsense_config,
                'camera_name': 'camera',
                'camera_namespace': 'camera'
            }.items()
        ),

        # Launch Point Selector Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_selector_launch),
            launch_arguments={
                'params_file': point_selector_config
            }.items()
        ),

        # Launch Path Planner Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_planner_launch),
            launch_arguments={
                'params_file': path_planner_config
            }.items()
        )
    ])
    