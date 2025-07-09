import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysl',
    maintainer_email='dldbstjd5566@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'localization = localization.localization:main',
            'odometry = localization.odometry:main',
            'encoder = localization.encoder_to_vel:main',
            'odometry_ekf = localization.odometry_ekf:main',
            'autocar_tf = localization.autocar_tf_ros2:main',
            'global_planner = localization.global_click_planner:main',
        ],
    },
)
