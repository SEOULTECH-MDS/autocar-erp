from setuptools import setup

package_name = 'dummy'

setup(
    name=package_name,
    version='0.0.0',
    # ament_python이 lib/<pkg> 구조와 런치에서 찾는 libexec 경로를 생성할 수 있도록 패키지를 설치
    packages=[package_name],
    package_dir={
        package_name: '.',
    },
    # 모듈은 패키지(dummy) 네임스페이스 하위로 설치됩니다
    data_files=[
        # ament index 패키지 마커
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/test_parking_area.launch.py',
            'launch/cones_pipeline.launch.py',
        ]),
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
            'cones_node = dummy.cones_node:main',
            'parking_area_node = dummy.parking_area_node:main',
            'initial_pose_node = dummy.initial_pose_node:main',
            'odometry_node = dummy.odometry_node:main',
            'road_node = dummy.road_node:main',
            'cone_map_node = dummy.cone_map_node:main',
            'rubber_cones_adapter_node = dummy.rubber_cones_adapter_node:main',
        ],
    },
)
