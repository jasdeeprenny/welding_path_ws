from setuptools import find_packages, setup

package_name = 'point_selector_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasdeep_renny',
    maintainer_email='jasrenny@hotmail.com',
    description='Captures two 2D clicks from the realsense 2 camera display and converts them to two 3D coordinate points',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_selector = point_selector_gui.point_selector_node:main'
        ],
    },
)
