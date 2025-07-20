from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.1',
    packages=['bae_planning', 'mode_selector', 'scenario_planner', 'planning_msgs', 'test_utils'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'src/test_utils/simple_test_launch.py',
            'src/test_utils/dynamic_test_launch.py',
            'src/test_utils/mode_selector_test_launch.py',
            'src/test_utils/visualization_launch.py',
            'src/test_utils/complete_visualization_launch.py'
        ]),
        ('share/' + package_name + '/rviz', [
            'src/test_utils/mode_selector.rviz'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BAE Team',
    maintainer_email='bae@example.com',
    description='Path Planning Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hybrid_astar_planner = bae_planning.hybrid_astar_planner:main',
            'zone_manager = scenario_planner.zone_manager:main',
            'mode_selector = mode_selector.mode_selector_node:main',
            'local_planner = bae_planning.local_planner_node:main',
            'plot_node = bae_planning.plot_node:main',
            'mode_selector_test = test_utils.mode_selector_test_node:main',
            'mode_selector_visualizer = test_utils.mode_selector_visualizer:main',
            'dynamic_vehicle_test = test_utils.dynamic_vehicle_test:main',
            'enhanced_visualizer = test_utils.enhanced_visualizer:main',
            'mode_selector_simple = mode_selector.mode_selector_node_simple:main',
            'simple_test = test_utils.simple_test_node:main',
            'tf_broadcaster = test_utils.tf_broadcaster:main',
        ],
    },
) 