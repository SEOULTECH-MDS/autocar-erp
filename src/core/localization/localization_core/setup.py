import os
from glob import glob
from setuptools import setup

package_name = 'localization_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['../resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*.osm')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysl',
    maintainer_email='dldbstjd5566@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = localization_core.localization:main',
            'odometry = localization_core.odometry:main',
            'encoder = localization_core.encoder_to_vel:main',
            'odometry_ekf = localization_core.odometry_ekf:main',
            'autocar_tf = localization_core.autocar_tf_ros2:main',
            'global_planner = localization_core.global_click_planner:main',
        ],
    },
) 