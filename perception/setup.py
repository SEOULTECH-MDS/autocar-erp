from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/perception']),
    ('share/perception', ['package.xml']),
    ('share/perception/models', ['perception/lanenet/models/culane_18.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkny2003',
    maintainer_email='kkny2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        	'stopline_camera = perception.stopline_camera:main',
        	'stopline_detection = perception.stopline_detection:main',

            'camera_pub = perception.lanenet.camera_pub:main',
            'lanenet = perception.lanenet.lanenet:main',

            # 'camera_traffic = perception.yolov7.camera_traffic:main',
            # 'trafficlight = perception.yolov7.trafficlight:main',

            # 'camera_obstacle = perception.yolov7.camera_obstacle:main',
            # 'combined_camera = perception.yolov7.obstacle_camera.combined_camera:main',
            # 'obstacle = perception.yolov7.obstacle:main',

            # 'camera_sign = perception.yolov7.camera_sign:main',
            # 'sign = perception.yolov7.sign:main',
            

            'camera_traffic = perception.yolov11.camera_traffic:main',
            'trafficlight = perception.yolov11.trafficlight:main',

            # 'camera_obstacle = perception.yolov11.camera_obstacle:main',
            'combined_camera = perception.yolov11.obstacle_camera.combined_camera:main',
            'obstacle = perception.yolov11.obstacle:main',

            'camera_sign = perception.yolov11.camera_sign:main',
            'sign = perception.yolov11.sign:main',
        ],
    },
)
