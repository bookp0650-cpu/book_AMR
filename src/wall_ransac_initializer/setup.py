from setuptools import find_packages, setup

package_name = 'wall_ransac_initializer'

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
    maintainer='shoseki',
    maintainer_email='simazakishota@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'wall_ransac_initialpose = wall_ransac_initializer.wall_ransac_initialpose:main',
                    'map_error_y = wall_ransac_initializer.map_error_y:main',
                    'map_error_x = wall_ransac_initializer.map_error_x:main',
        ],
    },
)
