# BAE Planning Package

Behavioral Autonomous Engineering (BAE) Planning Package는 자율주행을 위한 다양한 경로 계획 알고리즘과 도구들을 포함하는 ROS2 패키지입니다.

## 구조

```
BAE_clean/
├── src/
│   ├── bae_planning/           # 메인 계획 패키지
│   │   ├── hybrid_astar/       # Hybrid A* 알고리즘
│   │   ├── frenet_planner/     # Frenet 프레임 기반 계획
│   │   └── SHA/               # SHA (Simple Hybrid A*)
│   ├── scenario_planner/       # 시나리오 기반 계획
│   └── mode_selector/         # 모드 선택기
├── resource/
├── setup.py
├── package.xml
└── requirements.txt
```

## 주요 기능

### 1. Hybrid A* Path Planning
- 자동차의 동역학을 고려한 경로 계획
- Reeds-Shepp 경로를 이용한 최적화
- 동적 프로그래밍 휴리스틱

### 2. Frenet Frame Planning
- Frenet 프레임 기반 최적 궤적 생성
- Quintic 다항식 계획
- Cubic spline 기반 경로 생성

### 3. Zone Management
- 구역 기반 시나리오 관리
- 차량 위치 추적 및 구역 진입 감지
- 시각화 마커 발행

### 4. Mode Selection
- 다양한 주행 모드 관리
- 시나리오 기반 목표 설정

## 설치

### 요구사항
- ROS2 (Foxy 이상)
- Python 3.8+
- 필요한 Python 패키지들

### 설치 방법

1. **의존성 설치:**
```bash
pip install -r requirements.txt
```

2. **패키지 빌드:**
```bash
colcon build
```

3. **소스 설정:**
```bash
source install/setup.bash
```

## 사용법

### Hybrid A* 플래너 실행
```bash
ros2 run bae_planning hybrid_astar_planner
```

### Zone Manager 실행
```bash
ros2 run bae_planning zone_manager
```

### Mode Selector 실행
```bash
ros2 run bae_planning mode_selector
```

## 토픽

### 구독 토픽
- `/vehicle_pose` (geometry_msgs/PoseStamped): 차량 현재 위치
- `/scenario_mode` (std_msgs/String): 시나리오 모드

### 발행 토픽
- `/planned_path` (nav_msgs/Path): 계획된 경로
- `/zone_markers` (visualization_msgs/Marker): 구역 시각화

## 라이선스

MIT License

## 기여

이슈나 풀 리퀘스트를 통해 기여해주세요. 