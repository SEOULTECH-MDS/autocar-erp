"""
Setup for carla_twist_to_control
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'carla_twist_to_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.launch.py'))
                ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='CARLA twist to control for ROS2 bridge',
    license='MIT',
    entry_points={
        'console_scripts': ['carla_twist_to_control = carla_twist_to_control.carla_twist_to_control:main'],
    },
)
