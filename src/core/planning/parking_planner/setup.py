from setuptools import setup

package_name = 'parking_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/parking_planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Parking planner (empty scaffold).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_rubber_cones_adapter = parking_planner.parking_rubber_cones_adapter:main',
            'parking_cones_mapper = parking_planner.parking_cones_mapper:main',
            'parking_area_detector = parking_planner.parking_area_detector:main',
        ],
    },
)

