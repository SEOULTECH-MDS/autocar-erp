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
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
        (os.path.join('share', package_name, 'data/mirae_map'), glob('data/mirae_map/*')),
        (os.path.join('share', package_name, 'data/sample_map'), glob('data/sample_map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hmm',
    maintainer_email='hmm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autocar_tf_publisher = localization_core.autocar_tf_ros2:main',
            'global_planner = localization_core.global_planner:main',
            'global_click_planner = localization_core.global_click_planner:main',
            'map_visualizer = localization_core.map_visualizer:main',
            'local_planner = localization_core.local_planner:main',
            'path_follower = localization_core.path_follower:main',
            'path_selector = localization_core.path_selector:main',
            'odometry_ekf = localization_core.odometry_ekf:main',
            'local_obstacle_detector = localization_core.local_obstacle_detector:main',
            'local_path_publisher = localization_core.local_path_publisher:main',
        ],
    },
) 