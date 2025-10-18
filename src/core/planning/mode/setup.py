from setuptools import setup

package_name = 'mode'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Mode Selector v2.0 for autonomous driving',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_selector = mode.selector:main',
        ],
    },
)