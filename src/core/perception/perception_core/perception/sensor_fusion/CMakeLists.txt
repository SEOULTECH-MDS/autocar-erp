cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

############################
# 1. 필수 패키지
############################
find_package(ament_cmake          REQUIRED)
find_package(ament_cmake_python   REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 런타임(파이썬 노드)에서 쓰는 메시지·라이브러리
find_package(rclpy                REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(visualization_msgs   REQUIRED)
find_package(message_filters      REQUIRED)
find_package(cv_bridge            REQUIRED)        # OpenCV vendor

############################################
# 2.  커스텀 메시지(IDL) 생성
############################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  DEPENDENCIES std_msgs            # msg 안에서 사용하는 패키지
)

############################################
# 3.  파이썬 모듈/콘솔 스크립트 설치
############################################
ament_python_install_package(${PROJECT_NAME})      # setup.py/setup.cfg 기반

# (선택) launch 파일 등 리소스 설치
install(DIRECTORY launches
        DESTINATION share/${PROJECT_NAME})

############################################
# 4.  의존성 내보내기
############################################
ament_export_dependencies(
  rosidl_default_runtime           # 실행 시점에 메시지 사용
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
)

ament_package()