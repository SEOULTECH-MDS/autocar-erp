# Lane-based Mission Control System

## 개요

이 시스템은 kcity 맵에서 현재 lane ID를 기반으로 미션별 perception 로직을 선택적으로 실행하여 성능을 최적화합니다.

## 주요 구성 요소

### 1. Lane Mission Controller (`lane_mission_controller.py`)
- localization에서 제공하는 현재 lane ID를 구독
- lane ID에 따라 각 미션별 enable/disable 신호 발행
- 설정 파일을 통한 lane-mission 매핑 관리

### 2. Mission Config (`mission_config.yaml`)
- 미션별 lane ID 범위 정의
- 신호등: lane 10-15 (교차로 구간)
- 표지판: lane 20-25 (배달 구역)
- 장애물: lane 30-40 (장애물 회피 구간)
- 라바콘: lane 50-60 (주차 구간)

### 3. Enable 지원 Perception 노드들
기존 YOLO 탐지 노드들에 enable 로직 추가:
- `trafficlight.py`: 신호등 탐지
- `person_detect.py`: 사람 탐지
- `car_detect.py`: 차량 탐지
- `sign.py`: 표지판 탐지
- `rubber_detect.py`: 라바콘 탐지

센서퓨전 노드들에 enable 로직 추가:
- `sensor_fusion_sign.py`: 표지판(배달) 센서퓨전
- `sensor_fusion_obstacle.py`: 장애물(드럼통, 차량) 센서퓨전
- `sensor_fusion_rubber.py`: 라바콘(주차) 센서퓨전

## 토픽 구조

### 입력 토픽
- `/current_lane_id` (Int64): 현재 lane ID

### 출력 토픽 (Lane Mission Controller → Perception 노드들)
- `/mission/trafficlight/enable` (Bool): 신호등 미션 활성화
- `/mission/sign/enable` (Bool): 배달 미션 활성화
- `/mission/obstacle/enable` (Bool): 장애물 미션 활성화
- `/mission/rubber/enable` (Bool): 주차 미션 활성화

## 사용법

### 1. 빌드
```bash
cd /home/kkny2003/mds/autocar-erp
colcon build --packages-select perception
source install/setup.bash
```

### 2. 실행
```bash
# 전체 perception 시스템 실행 (lane mission controller 포함)
ros2 launch perception total.launch.py

# 또는 개별 노드 실행
ros2 run perception lane_mission_controller
ros2 run perception mock_localization  # 테스트용
```

### 3. 테스트
mock_localization 노드를 실행하면 다음과 같은 시퀀스로 lane ID가 변경됩니다:
```
Lane 5  -> 미션 외 구간
Lane 12 -> 신호등 미션 활성화
Lane 13 -> 신호등 미션 계속
Lane 18 -> 미션 외 구간
Lane 22 -> 표지판 미션 활성화
Lane 23 -> 표지판 미션 계속
Lane 28 -> 미션 외 구간
Lane 35 -> 장애물 미션 활성화
Lane 38 -> 장애물 미션 계속
Lane 45 -> 미션 외 구간
Lane 55 -> 라바콘 미션 활성화
Lane 58 -> 라바콘 미션 계속
Lane 65 -> 미션 외 구간
```

### 4. 실제 운용 시 설정

실제 localization 시스템과 연동할 때:
1. `total.launch.py`에서 mock_localization 노드 제거
2. localization 노드가 `/current_lane_id` 토픽 발행하도록 설정
3. `mission_config.yaml`에서 실제 kcity 맵의 lane ID에 맞게 범위 수정

## 설정 파일 수정

`config/mission_config.yaml`에서 lane ID 범위를 수정할 수 있습니다:

```yaml
mission_mapping:
  trafficlight:
    start: 10    # 시작 lane ID
    end: 15      # 종료 lane ID
  sign:
    start: 20
    end: 25
  obstacle:
    start: 30
    end: 40
  rubber:
    start: 50
    end: 60
```

## 성능 최적화 효과

1. **CPU 사용량 감소**: 불필요한 미션의 YOLO 추론을 건너뛰어 CPU 부하 감소
2. **메모리 사용량 감소**: 비활성화된 미션의 메모리 할당 최소화
3. **네트워크 트래픽 감소**: 빈 결과 메시지 발행으로 불필요한 데이터 전송 최소화
4. **실시간성 향상**: 필요한 미션만 실행하여 전체 시스템 응답 시간 개선

## 로그 확인

각 노드는 활성화/비활성화 상태 변경 시 로그를 출력합니다:
```
[INFO] [lane_mission_controller]: trafficlight 미션 활성화 (Lane 12)
[INFO] [trafficlight]: 신호등 탐지 활성화
[INFO] [sensor_fusion_sign]: 표지판 센서퓨전 비활성화
```

## 문제 해결

1. **Enable 신호가 전달되지 않는 경우**:
   - lane_mission_controller 노드가 실행 중인지 확인
   - `/localization/current_lane_id` 토픽이 발행되고 있는지 확인

2. **설정 파일을 찾을 수 없는 경우**:
   - `colcon build` 후 `source install/setup.bash` 실행 확인
   - `config/mission_config.yaml` 파일 존재 확인

3. **특정 미션이 활성화되지 않는 경우**:
   - `mission_config.yaml`의 lane ID 범위 확인
   - 해당 perception 노드의 enable 토픽 구독 확인
