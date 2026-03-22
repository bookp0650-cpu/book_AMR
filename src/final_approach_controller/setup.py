from setuptools import find_packages, setup

package_name = 'final_approach_controller'

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
            'final_approach_node = final_approach_controller.final_approach_node:main',
            'final_approach_node2 = final_approach_controller.final_approach_node2:main',
        ],
    },
)
