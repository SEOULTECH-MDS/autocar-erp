from setuptools import setup, find_packages

package_name = 'bae_planning'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BAE Team',
    maintainer_email='bae@example.com',
    description='Behavioral Autonomous Engineering Planning Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hybrid_astar_planner = bae_planning.hybrid_astar_planner:main',
            'zone_manager = scenario_planner.zone_manager:main',
            'mode_selector = mode_selector.mode_selector_node:main',
            'local_planner = bae_planning.local_planner_node:main',
            'plot_node = bae_planning.plot_node:main',
        ],
    },
) 