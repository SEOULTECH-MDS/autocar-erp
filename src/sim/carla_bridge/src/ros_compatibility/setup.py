from setuptools import setup, find_packages

package_name = 'ros_compatibility'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='The ros_compatibility package',
    license='MIT',
)
