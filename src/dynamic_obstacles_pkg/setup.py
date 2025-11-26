from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dynamic_obstacles_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob('configs/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajr',
    maintainer_email='stippinger.szabolcs@gmail.com',
    description='Dynamic Obstacles Spawner',
    license='Szabolcs Stippinger',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_obstacle = dynamic_obstacles_pkg.dynamic_obstacle:main',
            'dynamic_obstacle_to_map_update = dynamic_obstacles_pkg.dynamic_obstacle_to_map_update:main'
        ],
    },
)
