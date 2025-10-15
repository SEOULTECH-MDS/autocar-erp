from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception']),
        ('share/perception', ['package.xml']),

        # lanenet 모델 파일
        ('share/perception/models', ['perception/lanenet/models/culane_18.pth']),

        # ==============================
        # YOLO 가중치 파일 설치 부분 추가
        # perception/yolov11/weights/*.pt 파일을 모두 설치
        # ==============================
        (os.path.join('share', package_name, 'yolov11', 'weights'), glob('perception/yolov11/weights/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkny2003',
    maintainer_email='kkny2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # # 정지선
            # 'stopline_camera = perception.stopline_camera:main',
            # 'stopline_detection = perception.stopline_detection:main',

            # # 차선
            # 'camera_pub = perception.lanenet.camera_pub:main',
            # 'lanenet = perception.lanenet.lanenet:main',
            
            # 카메라
            'camera_front = perception.yolov11.camera_front:main', # 신호등, 장애물, 배달
            'camera_side = perception.yolov11.camera_side:main', # 주차 라바콘

            # 신호등
            'trafficlight = perception.yolov11.trafficlight:main',

            # 라바콘 (주차)
            'rubber_detect = perception.yolov11.rubber_detect:main',
            "sensor_fusion_rubber = perception.sensor_fusion.src.rubber.sensor_fusion:main",
            "bbox_tracker = perception.tracker.src.bbox_tracker:main",
            "object_tracker3D = perception.tracker.src.object_tracker3D:main",
            "rubber_visualizer = perception.rubber_visualizer.src.rubber_visualizer:main",

            # 장애물
            "car_detect = perception.yolov11.car_detect:main", # 차
            "drum_detect = perception.yolov11.drum_detect:main", # 드럼
            "person_detect = perception.yolov11.person_detect:main", # 사람
            "sensor_fusion_obstacle = perception.sensor_fusion.src.obstacle.sensor_fusion:main",

            # 표지판
            'sign = perception.yolov11.sign:main',
            "sensor_fusion_sign = perception.sensor_fusion.src.sign.sensor_fusion:main",
            # "sensor_fusion = perception.sensor_fusion.src.sensor_fusion_v3kcity:main",

            # 포인트클라우드 필터
            'region_filter_node = perception.pointcloud.region_filter_node:main',

            
        ],
    },
)
