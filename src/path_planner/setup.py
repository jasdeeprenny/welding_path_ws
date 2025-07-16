from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/path_planner_launch.py',
            'launch/point_selector_launch.py',
            'launch/full_system_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/point_selector_params.yaml',
            'config/realsense_config.yaml',
            'config/path_planner_params.yaml'
        ]),
    ],
    install_requires=[
        'setuptools', 
        'numpy', 
        'scikit-image', 
        'open3d', 
        'tf_transformations',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='jasdeep_renny',
    maintainer_email='jasrenn@hotmail.com',
    description='Plans a 3D path between two clicked pixel points',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = path_planner.path_planner_node:main',
            'point_selector = path_planner.point_selector_node:main',
        ],
    },
)