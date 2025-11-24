from setuptools import find_packages, setup

package_name = 'simulation_resources_pkg'

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
    maintainer='ajr',
    maintainer_email='stippinger.szabolcs@gmail.com',
    description='Maps generate package',
    license='Szabolcs Stippinger',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
