from setuptools import setup, find_packages

package_name = 'scenario_planner'

setup(
    name=package_name,
    version='0.0.0',
    # scenario_planner 패키지만 포함
    packages=find_packages(include=[
        'scenario_planner',
        'scenario_planner.*',
    ]),
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=[
        'setuptools',
        'numpy',
        'shapely',
        # Hybrid A*가 quaternion 변환이나 tf 를 쓰면 여기에 추가
    ],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='your@email.com',
    description='Scenario-based path planner using Hybrid A*',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_pose_publisher = scenario_planner.dummy_pose_publisher:main',
            'hybrid_astar_planner = scenario_planner.hybrid_astar_planner:main',
            'zone_manager = scenario_planner.zone_manager:main',
            'path_visualizer = scenario_planner.path_visualizer:main',
        ],
    },
)
