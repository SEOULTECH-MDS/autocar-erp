import os
from glob import glob
from setuptools import setup

package_name = 'localization_core'

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files([
        ('share/ament_index/resource_index/packages', ['../resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ], ['data/']),
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
            'localization = localization_core.localization:main',
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