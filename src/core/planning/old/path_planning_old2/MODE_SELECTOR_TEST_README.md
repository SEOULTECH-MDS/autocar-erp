# Mode Selector 테스트 및 맵 기반 미션 시스템 가이드

## 🎯 목적
Mode Selector가 센서 데이터에 따라 올바르게 모드를 변경하는지 시각적으로 확인하고, 맵 기반 미션 시스템을 통해 실제 주행 시나리오를 시뮬레이션합니다.

## 📋 시스템 구성

### 🧠 핵심 컴포넌트
1. **Mode Selector FSM** - 유한 상태 머신 기반 모드 선택
2. **Mission Map System** - 맵 기반 미션 포인트 관리
3. **Sensor Simulation** - 센서 데이터 시뮬레이션
4. **3D Visualization** - RViz 기반 시각화
5. **Automated Scripts** - 자동화된 실행 스크립트

### 🗺️ 미션 타입
- **정지선 (Stop Line)** - 정지선 감지 시 일시 정지
- **장애물 (Obstacle)** - 장애물 감지 시 회피 모드
- **배달 표지판 (Delivery Sign)** - 배달 미션 수행
- **주차 라바콘 (Parking Cone)** - 주차 미션 수행
- **정상 주행 (Normal Driving)** - 기본 주행 모드

## 🚀 실행 방법

### 방법 1: 맵 기반 미션 시스템 (권장)
```bash
# 프로젝트 루트에서 실행
cd /home/kmkm/autocar-erp
./src/core/planning/path_planning/scripts/run_mission_map_final.sh
```

### 방법 2: 기본 시각화 시스템
```bash
# 기본 테스트 시나리오 실행
cd /home/kmkm/autocar-erp
./src/core/planning/path_planning/scripts/run_visualization.sh
```

### 방법 3: 수동 실행
```bash
# 1. 패키지 빌드
cd /home/kmkm/autocar-erp
colcon build --packages-select path_planning

# 2. 환경 설정
source install/setup.bash

# 3. 개별 노드 실행
install/path_planning/bin/tf_broadcaster &
install/path_planning/bin/mode_selector_simple &
install/path_planning/bin/mission_map &
install/path_planning/bin/mode_selector_visualizer &
rviz2 -d src/core/planning/path_planning/src/test_utils/mode_selector.rviz &
```

## 📁 스크립트 파일 설명

### `/scripts/` 폴더의 스크립트들:

1. **`run_mission_map_final.sh`** ⭐ **주요 스크립트**
   - 맵 기반 미션 시스템 (완전 자동화)
   - 차량이 맵을 주행하며 미션 수행
   - 실시간 3D 시각화 및 모니터링

2. **`run_visualization.sh`**
   - 기본 시각화 시스템
   - 순환 테스트 시나리오

3. **`run_mission_map.sh`**
   - 맵 기반 미션 시스템 (기본 버전)

4. **`run_mission_map_fixed.sh`**
   - 맵 기반 미션 시스템 (수정 버전)

## 🗺️ 맵 기반 미션 시스템

### 📍 미션 포인트 배치
```
0m → 10m → 20m → 30m → 40m → 50m
🚗   🛑   🚧   📦   🅿️   🚗
정상  정지  장애  배달  주차  정상
주행  선   물   미션  미션  주행
```

### 🎯 미션 시나리오
1. **0-10m**: 정상 주행 (Drive Mode)
2. **10m**: 정지선 감지 (Pause Mode)
3. **20m**: 장애물 감지 (Obstacle Mode)
4. **30m**: 배달 표지판 (Delivery Mode)
5. **40m**: 주차 라바콘 (Parking Mode)
6. **50m+**: 정상 주행 복귀 (Drive Mode)

### 🚗 차량 동작
- **속도**: 2m/s 일정 속도 주행
- **경로**: 직선 주행 (X축 방향)
- **미션 인식**: 3m 반경 내 도달 시 자동 인식
- **완료 조건**: 모드 전환 시 미션 완료 처리

## 📊 시각화 요소

### RViz에서 확인할 수 있는 요소들:

#### 🎨 미션 포인트 마커
- **🔴 빨간색 구체**: 정지선 미션
- **🔵 파란색 구체**: 장애물 미션
- **🟣 보라색 구체**: 배달 미션
- **🟠 주황색 구체**: 주차 미션
- **🟢 초록색 구체**: 정상 주행 구간
- **🟡 노란색**: 현재 활성화된 미션
- **⚫ 회색**: 완료된 미션

#### 🚗 차량 및 경로
- **🟡 노란색 선**: 차량 주행 경로
- **🚗 3D 차량 모델**: 모드별 색상 변화
- **📝 텍스트 오버레이**: 현재 모드 및 설명

#### 📊 실시간 정보
- **모드 상태**: 현재 활성 모드
- **미션 진행률**: 완료된 미션 수
- **차량 위치**: 실시간 좌표

## 📝 콘솔 출력 확인

### 맵 기반 미션 시스템 로그:
```
🗺️ Mission Map 시작됨
📋 6개의 미션 포인트 설정됨
📍 차량 위치: (10.2, 0.0) | 완료된 미션: 1/6
✅ 미션 완료: stop_line at (10.0, 0.0)
🔄 모드 변경: Pause Mode: 정지선 가까움
```

### 모드 셀렉터 로그:
```
🧠 Mode Selector Simple 노드가 시작되었습니다
[FSM] Current Mode: Drive Mode: 정상 주행 (State: 0)
받은 정지선 거리: 2.50m
[FSM] Mode Changed: Pause Mode: 정지선 가까움
```

## 🔧 문제 해결

### 1. 스크립트 실행 오류
```bash
# 실행 권한 확인
chmod +x src/core/planning/path_planning/scripts/*.sh

# 프로젝트 루트에서 실행
cd /home/kmkm/autocar-erp
./src/core/planning/path_planning/scripts/run_mission_map_final.sh
```

### 2. 빌드 오류
```bash
# 패키지 재빌드
colcon build --packages-select path_planning
source install/setup.bash
```

### 3. 노드 실행 오류
```bash
# 기존 프로세스 정리
pkill -f tf_broadcaster
pkill -f mode_selector_simple
pkill -f mission_map
pkill -f mode_selector_visualizer
pkill -f rviz2
```

### 4. RViz 문제
```bash
# RViz 설정 파일 확인
ls src/core/planning/path_planning/src/test_utils/mode_selector.rviz

# 수동 RViz 실행
rviz2 -d src/core/planning/path_planning/src/test_utils/mode_selector.rviz
```

## 📈 테스트 결과 해석

### ✅ 정상 동작 시:
- 차량이 미션 포인트를 순차적으로 통과
- 각 미션에서 적절한 모드로 자동 전환
- 미션 완료 시 시각적 피드백 (회색으로 변경)
- 실시간 경로 추적 및 3D 시각화

### ❌ 문제가 있는 경우:
- 미션 포인트가 인식되지 않음
- 모드 전환이 발생하지 않음
- 시각화 요소가 표시되지 않음
- 스크립트가 즉시 종료됨

## 🎯 고급 테스트

### 1. 토픽 모니터링
```bash
# 모드 상태 모니터링
ros2 topic echo /mode_state

# 센서 데이터 모니터링
ros2 topic echo /stop_line_distance
ros2 topic echo /obstacle_detected
ros2 topic echo /sign_detector
ros2 topic echo /cone_detector

# 미션 마커 모니터링
ros2 topic echo /mission_markers
```

### 2. 수동 센서 데이터 발행
```bash
# 정지선 거리 수동 설정
ros2 topic pub /stop_line_distance std_msgs/msg/Float32 "data: 2.0"

# 장애물 감지 수동 설정
ros2 topic pub /obstacle_detected std_msgs/msg/Bool "data: true"

# 배달 표지판 수동 설정
ros2 topic pub /sign_detector std_msgs/msg/String "data: 'DELIVERY'"
```

### 3. 시스템 상태 확인
```bash
# 실행 중인 노드 확인
ros2 node list

# 활성 토픽 확인
ros2 topic list

# 노드 정보 확인
ros2 node info /mode_selector_simple
ros2 node info /mission_map
```

## 🚀 다른 컴퓨터에서 실행

### Git 클론 후 실행:
```bash
# 1. 프로젝트 클론
git clone <repository-url>
cd autocar-erp

# 2. 빌드
colcon build --packages-select path_planning

# 3. 환경 설정
source install/setup.bash

# 4. 스크립트 실행
./src/core/planning/path_planning/scripts/run_mission_map_final.sh
```

### 스크립트 특징:
- **자동 경로 감지**: 어디서 실행하든 프로젝트 루트를 자동으로 찾음
- **절대 경로 사용**: 모든 파일 경로를 절대 경로로 처리
- **에러 처리**: 각 단계별 상세한 에러 메시지
- **프로세스 모니터링**: 실시간 노드 상태 확인

## 📚 추가 정보

### 📁 파일 구조:
```
path_planning/
├── scripts/                    # 실행 스크립트
│   ├── run_mission_map_final.sh
│   ├── run_visualization.sh
│   ├── run_mission_map.sh
│   └── run_mission_map_fixed.sh
├── src/
│   ├── mode_selector/          # 모드 셀렉터 노드
│   └── test_utils/            # 테스트 유틸리티
│       ├── mission_map.py     # 맵 기반 미션 시스템
│       ├── mode_selector_visualizer.py
│       ├── tf_broadcaster.py
│       └── mode_selector.rviz
└── MODE_SELECTOR_TEST_README.md
```

### 🔗 관련 토픽:
- `/mode_state` - 현재 모드 상태
- `/mode_description` - 모드 설명
- `/mission_markers` - 미션 포인트 마커
- `/vehicle_path` - 차량 주행 경로
- `/stop_line_distance` - 정지선 거리
- `/obstacle_detected` - 장애물 감지
- `/sign_detector` - 표지판 감지
- `/cone_detector` - 라바콘 감지

---

**작성자**: BAE Team  
**버전**: 2.0  
**최종 업데이트**: 2024년 12월 