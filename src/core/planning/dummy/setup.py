from setuptools import setup

package_name = 'dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_parking_area.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/parking_area.rviz']),
    ],
    install_requires=['setuptools', 'numpy>=1.21.0', 'scikit-learn>=1.0.0'],
    zip_safe=True,
    maintainer='Test User',
    maintainer_email='test@example.com',
    description='Dummy package for testing cones visualization',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cones_node = cones_node:main',
            'parking_area_node = parking_area_node:main',
        ],
    },
)
