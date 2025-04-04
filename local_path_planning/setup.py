from setuptools import find_packages, setup

package_name = 'local_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.path_finder'], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  
    zip_safe=True,
    maintainer='ysl',
    maintainer_email='dldbstjd5566@gmail.com',
    description='Local path planning package',
    license='Apache License 2.0', 
    entry_points={
        'console_scripts': [
            'optimal_frenet_planning = local_path_planning.optimal_frenet_planning:main',
        ],
    },
)