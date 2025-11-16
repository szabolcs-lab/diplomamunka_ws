from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'd_star_lite_pkg'

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
    description='D* Lite implementation in ROS2',
    license='Szabolcs Stippinger',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publication = d_star_lite_pkg.map_publication:main',
            'd_star_lite_path_planner = d_star_lite_pkg.d_star_lite_path_planner:main',
            'nav2_path = d_star_lite_pkg.nav2_path:main',
            'tf_broadcaster = d_star_lite_pkg.tf_broadcaster:main'
        ],
    },
)
