from setuptools import setup

package_name = 'planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_planner.launch.py']),
    ],
    install_requires=['setuptools', 'numpy>=1.21.0'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Parking path planner using Hybrid A* algorithm',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = planner:main',
        ],
    },
)
