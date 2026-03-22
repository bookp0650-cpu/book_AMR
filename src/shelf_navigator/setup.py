from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'shelf_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ config を install する
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shoseki',
    maintainer_email='shoseki@todo.todo',
    description='Shelf ID based navigation logic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shelf_id_logic = shelf_navigator.nodes.shelf_id_logic:main',
            'shelf_id_logic2 = shelf_navigator.nodes.shelf_id_logic2:main',
        ],
    },
)
