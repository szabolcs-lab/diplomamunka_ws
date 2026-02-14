from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hibrid_ai_pkg'

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
    install_requires=['setuptools','torch'],
    zip_safe=True,
    maintainer='ajr',
    maintainer_email='ajr@todo.todo',
    description='Hibrid - D* Lite - NAV2 - PPO',
    license='Szabolcs Stippinger',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publication = hibrid_ai_pkg.map_publication:main',
            'd_star_lite_path_planner = hibrid_ai_pkg.d_star_lite_path_planner:main',
            'nav2_path = hibrid_ai_pkg.nav2_path:main',
            'tf_broadcaster = hibrid_ai_pkg.tf_broadcaster:main',
            'trajectory_smoother = hibrid_ai_pkg.trajectory_smoother:main',
            'ppo_trainer = hibrid_ai_pkg.ppo_trainer:main',
            'ppo_product = hibrid_ai_pkg.ppo_productr:main'
        ],
    },
)
