from setuptools import setup

package_name = 'erp_control'

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
    maintainer='mds2',
    maintainer_email='mds2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        'erp_publish = erp_control.erp_publish:main',
        'erp_client = erp_control.erp_client:main',
        'ERP42_ros2 = erp_control.ERP42_ros2:main',
        'simul_plot = erp_control.simul_plot:main',
        'delay = erp_control.delay_test:main',
        'erp_test = erp_control.erp42_ros2_test:main',
        ],
    },
)
