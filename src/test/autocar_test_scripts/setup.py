from setuptools import find_packages, setup

package_name = 'autocar_test_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysl',
    maintainer_email='dldbstjd5566@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'hitech_test_pub = autocar_test_scripts.hitech_test_pub:main',
            'simulation_pub = autocar_test_scripts.simulation_pub:main',
        ],
    },
) 