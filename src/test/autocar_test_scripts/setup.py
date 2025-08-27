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
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='ysl',
    maintainer_email='dldbstjd5566@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'hitech_test_pub = autocar_test_scripts.hitech_test_pub:main',
            'simulation_pub = autocar_test_scripts.simulation_pub:main',
            'mode_pub = autocar_test_scripts.mode_pub:main',
            'dummy_local_path_pub = autocar_test_scripts.dummy_local_path_pub:main',
            'mpc_realtime_plotter = autocar_test_scripts.mpc_realtime_plotter:main',
        ],
    },
) 