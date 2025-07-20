# Mode Selector 테스트 가이드

## 🎯 목적
Mode Selector가 센서 데이터에 따라 올바르게 모드를 변경하는지 시각적으로 확인합니다.

## 📋 테스트 시나리오

### 1. 정상 주행 (0-5초)
- **센서 데이터**: 정지선 거리 50m, 장애물 없음, 표지판 없음
- **예상 모드**: DRIVE (녹색)

### 2. 정지선 감지 (5-8초)
- **센서 데이터**: 정지선 거리가 3m 이하로 점진적으로 감소
- **예상 모드**: PAUSE (노란색)

### 3. 장애물 감지 (8-11초)
- **센서 데이터**: 장애물 감지됨, 정지선 거리 50m
- **예상 모드**: OBSTACLE_STATIC (빨간색)

### 4. 배달 표지판 감지 (11-16초)
- **센서 데이터**: 배달 표지판 감지됨
- **예상 모드**: DELIVERY (파란색)

### 5. 주차 라바콘 감지 (16-21초)
- **센서 데이터**: 주차 라바콘 감지됨
- **예상 모드**: PARKING (마젠타)

### 6. 정상 주행 복귀 (21-24초)
- **센서 데이터**: 모든 센서 정상
- **예상 모드**: DRIVE (녹색)

## 🚀 실행 방법

### 방법 1: 스크립트 실행 (권장)
```bash
cd /home/kmkm/autocar-erp
./src/core/planning/path_planning/test_mode_selector.sh
```

### 방법 2: 수동 실행
```bash
# 1. 패키지 빌드
cd /home/kmkm/autocar-erp
colcon build --packages-select path_planning planning_msgs

# 2. 환경 설정
source install/setup.bash

# 3. Launch 파일 실행
ros2 launch path_planning mode_selector_test_launch.py
```

## 📊 시각화 요소

### RViz에서 확인할 수 있는 요소들:

1. **모드 상태 원통** (차량 앞쪽)
   - 🟢 녹색: DRIVE 모드
   - 🔴 빨간색: OBSTACLE_STATIC 모드
   - 🟡 노란색: PAUSE 모드
   - 🔵 파란색: DELIVERY 모드
   - 🟣 마젠타: PARKING 모드

2. **모드 이름 텍스트** (원통 위)
   - 현재 모드의 영문 이름 표시

3. **모드 설명 텍스트** (이름 위)
   - 현재 모드의 한글 설명 표시

4. **센서 시뮬레이션**
   - 🔴 빨간 선: 정지선 위치
   - 🔴 빨간 구: 장애물 (OBSTACLE 모드에서만)
   - 🔵 파란 큐브: 배달 표지판 (DELIVERY 모드에서만)
   - 🟣 마젠타 원뿔: 주차 라바콘 (PARKING 모드에서만)

## 📝 콘솔 출력 확인

터미널에서 다음과 같은 로그를 확인할 수 있습니다:

```
📋 현재 시나리오: Normal Driving (시간: 0.0s)
🔄 모드 변경: Drive Mode: 정상 주행
📋 현재 시나리오: Stop Line Detection (시간: 5.0s)
🔄 모드 변경: Pause Mode: 정지선 가까움
📋 현재 시나리오: Obstacle Detection (시간: 8.0s)
🔄 모드 변경: Obstacle Detected: 정적 장애물 회피 모드
...
```

## 🔧 문제 해결

### 1. 빌드 오류
```bash
# 의존성 설치
sudo apt update
sudo apt install python3-pip
pip install -r requirements.txt
```

### 2. 메시지 타입 오류
```bash
# planning_msgs 패키지 재빌드
colcon build --packages-select planning_msgs
source install/setup.bash
```

### 3. RViz가 열리지 않는 경우
```bash
# RViz 수동 실행
ros2 run rviz2 rviz2
```

## 📈 테스트 결과 해석

### 정상 동작 시:
- 각 시나리오에서 예상된 모드로 정확히 전환
- 모드 전환 시 적절한 색상 변화
- 콘솔에 모드 변경 로그 출력

### 문제가 있는 경우:
- 모드가 변경되지 않음
- 잘못된 모드로 전환
- 시각화 요소가 표시되지 않음

## 🎯 추가 테스트 아이디어

1. **수동 센서 데이터 발행**
```bash
# 정지선 거리 수동 설정
ros2 topic pub /stop_line_distance std_msgs/msg/Float32 "data: 2.0"

# 장애물 감지 수동 설정
ros2 topic pub /obstacle_detected std_msgs/msg/Bool "data: true"
```

2. **토픽 모니터링**
```bash
# 모드 상태 토픽 모니터링
ros2 topic echo /mode_state

# 센서 데이터 토픽 모니터링
ros2 topic echo /stop_line_distance
ros2 topic echo /obstacle_detected
```

3. **시각화 마커 확인**
```bash
# 시각화 마커 토픽 모니터링
ros2 topic echo /mode_visualization
``` 