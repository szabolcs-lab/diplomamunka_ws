from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'a_star_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.csv')),
        (os.path.join('share', package_name, 'configs'), glob('configs/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajr',
    maintainer_email='ajr@todo.todo',
    description='A* implementaion is ROS2',
    license='Szabolcs Stippinger',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publication = a_star_pkg.map_publication:main',
            'a_star_path_planner = a_star_pkg.a_star_path_planner:main',
            'nav2_path = a_star_pkg.nav2_path:main',
            'tf_broadcaster = a_star_pkg.tf_broadcaster:main'
        ],
    },
)
