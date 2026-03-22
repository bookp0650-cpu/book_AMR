from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'udp_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@todo.todo',
    description='UDP/TCP bridge nodes for AMR',
    license='TODO',
    entry_points={
        'console_scripts': [
            'shelf_id_receiver = udp_bridge.shelf_id_receiver:main',
            'navigation_goal_sender = udp_bridge.navigation_goal_sender:main',
            'navigation_goal_final_sender = udp_bridge.navigation_goal_final_sender:main',
            'error_x_receiver = udp_bridge.error_x_receiver:main',
            'wall_distance_sender = udp_bridge.wall_distance_sender:main',
            'wall_yaw_deg_sender = udp_bridge.wall_yaw_deg_sender:main',
            'cmd_vel_receiver = udp_bridge.cmd_vel_receiver:main',
        ],
    },
)
