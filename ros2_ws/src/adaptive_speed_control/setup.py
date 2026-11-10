from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adaptive_speed_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger'), glob('models/turtlebot3_burger/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayush Kumar',
    maintainer_email='kumar@todo.todo',
    description='Adaptive Speed Control in Cluttered Environments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_speed_controller = adaptive_speed_control.adaptive_speed_controller_node:main',
            'clutter_navigator = adaptive_speed_control.clutter_navigator_node:main',
            'speed_telemetry_hud = adaptive_speed_control.speed_telemetry_hud:main',
        ],
    },
)
