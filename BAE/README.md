# BAE (Behavioral Autonomous Engineering)

이 패키지는 자율주행을 위한 행동 계획 및 경로 계획 시스템을 포함합니다.

## 구조

- `ROS2_Planning/`: ROS2 기반 경로 계획 시스템
- `ros2_ws/`: ROS2 워크스페이스
- `ros2_planning/`: ROS2 경로 계획 패키지

## 설치 및 실행

### 요구사항
- ROS2 (Foxy 이상)
- Python 3.8+
- 필요한 Python 패키지들

### 빌드
```bash
# ROS2 워크스페이스 빌드
cd ros2_ws
colcon build

# 소스 설정
source install/setup.bash
```

### 실행
```bash
# 경로 계획 노드 실행
ros2 run ros2_planning path_planner
```

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 