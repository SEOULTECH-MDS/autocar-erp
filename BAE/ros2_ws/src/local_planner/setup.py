import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['local_planner', 'local_planner.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 추가할 부분
        (os.path.join('share', package_name, 'global_paths'), glob('global_paths/*.csv')),
        ('share/local_planner/launch', ['launch/dual_node_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmkm',
    maintainer_email='kmkm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_planner_node = local_planner.local_planner_node:main',
            'dummy_input_node = local_planner.dummy_input_node:main',
            'mode_selector_node = local_planner.mode_selector_node:main',
            'plot_node = local_planner.plot_node:main',
        ],
    },
)
