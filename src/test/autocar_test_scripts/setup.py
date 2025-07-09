from setuptools import setup

package_name = 'autocar_test_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Test scripts for Autocar ERP',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hitech_test_pub = autocar_test_scripts.hitech_test_pub:main',
            'simulation_pub = autocar_test_scripts.simulation_pub:main',
        ],
    },
) 