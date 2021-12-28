from setuptools import setup
import os
from glob import glob

package_name = 'riptide_gazebo2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OSU UWRT',
    maintainer_email='osu.uwrt@gmail.com',
    description='Worlds, models, and code to interface with the robot simulator gazebo',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_remap = riptide_gazebo2.depth_remap:main',
            'imu_remap = riptide_gazebo2.imu_remap:main',
            'kill_switch_publisher = riptide_gazebo2.kill_switch_publisher:main',
            'thrust_remap = riptide_gazebo2.thrust_remap:main',
        ],
    },
)
