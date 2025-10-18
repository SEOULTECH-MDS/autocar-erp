# Mode Selector v2.0

자율주행 차량의 모드를 관리하는 ROS2 노드입니다.

## 📋 개요

Mode Selector는 차량의 현재 상태와 센서 입력을 기반으로 적절한 주행 모드를 선택하고 전환하는 핵심 노드입니다.

### 주요 특징

- ✅ **예선/본선 모드 분리**: 대회 타입에 따라 다른 모드 세트 사용
- ✅ **다중 맵 지원**: K-City와 미래관 맵을 모두 지원
- ✅ **신호등 기반 제어**: 안정적인 신호등 인식 및 0.3초 확인 시스템
- ✅ **특수 정지선 처리**: 우회전 구역 자동 정지 및 타이머 기반 출발
- ✅ **배달 미션 관리**: 상차/하차 시퀀스 자동 관리

---

## 🎯 지원 모드

### 예선 모드 (Preliminary)
| 모드 | ModeState | 설명 |
|------|-----------|------|
| PRELIMINARY_DRIVING | DRIVE (0) | 일반 주행 |
| PRELIMINARY_PARKING | PARKING (5) | 주차 미션 |
| PRELIMINARY_UTURN | UTURN (7) | 유턴 구역 |
| PRELIMINARY_GPS_OFF | GPS_OFF (8) | GPS 차단 구역 |

### 본선 모드 (Final)
| 모드 | ModeState | 설명 |
|------|-----------|------|
| FINAL_DRIVING | DRIVE (0) | 일반 주행 |
| FINAL_PAUSE | PAUSE (1) | 신호등/정지선 대기 |
| FINAL_PARKING | PARKING (5) | 주차 미션 |
| FINAL_DELIVERY_PICKUP | DELIVERY (4) | 배달 상차 |
| FINAL_DELIVERY_DROPOFF | DELIVERY (4) | 배달 하차 |

---

## 📡 토픽

### 구독 (Subscriptions)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/traffic_sign` | String | 신호등 상태 ("Green", "Red", "Left", "Straightleft") |
| `/current_lanelet_id` | Int64 | 현재 Lanelet ID |
| `/target_sign` | Int32 | 표지판 인식 (1-3: 상차, 4-6: 하차) |
| `/parking_complete_flag` | Bool | 주차 미션 완료 신호 |
| `/pickup_complete_flag` | Bool | 상차 미션 완료 신호 |
| `/delivery_complete_flag` | Bool | 하차 미션 완료 신호 |
| `/stopline_distance` | Float64 | 정지선까지의 거리 (m) |
| `/stopline_type` | String | 정지선 타입 ("right", "left", "straight", etc.) |
| `/autocar/location` | Odometry | 차량 위치 및 속도 |

### 발행 (Publications)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mode_state` | ModeState | 현재 모드 상태 및 설명 |

---

## ⚙️ 파라미터

### 기본 설정

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `competition_type` | string | "preliminary" | 대회 타입 ("preliminary" \| "final") |
| `map_type` | string | "kcity" | 맵 타입 ("kcity" \| "mirae") |
| `publish_rate` | double | 10.0 | 발행 주기 (Hz) |

### 신호등 및 정지선

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `traffic_signal_confirm_duration` | double | 0.3 | 신호등 전환 확인 시간 (초) |
| `stopline_pause_distance` | double | 5.0 | 정지선 감지 거리 (m) |
| `stopline_pause_duration` | double | 3.2 | 우회전 정지 대기 시간 (초) |
| `vehicle_stop_velocity_threshold` | double | 0.1 | 차량 정지 판단 속도 (m/s) |
| `enable_distance_condition` | bool | false | 거리 조건 사용 여부 |

### K-City 구역 설정

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `kcity_preliminary_parking_zones` | int[] | [1] | 예선 주차 구역 Lanelet ID |
| `kcity_final_parking_zones` | int[] | [21] | 본선 주차 구역 Lanelet ID |
| `kcity_uturn_zones` | int[] | [200, 201] | 유턴 구역 Lanelet ID |
| `kcity_gps_off_zones` | int[] | [300, 301] | GPS 차단 구역 Lanelet ID |
| `kcity_delivery_zones` | int[] | [1, 15] | 배달 구역 Lanelet ID |
| `kcity_right_pause_zones` | int[] | [9, 10] | 우회전 정지선 구역 Lanelet ID |

### 미래관 구역 설정

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `mirae_parking_zones` | int[] | [7] | 주차 구역 Lanelet ID |
| `mirae_uturn_zones` | int[] | [39] | 유턴 구역 Lanelet ID |
| `mirae_gps_off_zones` | int[] | [23] | GPS 차단 구역 Lanelet ID |
| `mirae_delivery_zones` | int[] | [28] | 배달 구역 Lanelet ID |
| `mirae_right_pause_zones` | int[] | [55, 60] | 우회전 정지선 구역 Lanelet ID |

---

## 🚀 사용법

### 기본 실행

```bash
# 빌드
cd ~/autocar-erp
colcon build --packages-select mode

# 환경 설정
source install/setup.bash

# 실행
ros2 run mode mode_selector
```

### 파라미터와 함께 실행

```bash
# 본선 모드, K-City 맵
ros2 run mode mode_selector --ros-args \
  -p competition_type:=final \
  -p map_type:=kcity

# 예선 모드, 미래관 맵
ros2 run mode mode_selector --ros-args \
  -p competition_type:=preliminary \
  -p map_type:=mirae

# 신호등 확인 시간 조정
ros2 run mode mode_selector --ros-args \
  -p traffic_signal_confirm_duration:=0.5
```

### Launch 파일 예시

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mode',
            executable='mode_selector',
            name='mode_selector',
            parameters=[{
                'competition_type': 'final',
                'map_type': 'kcity',
                'traffic_signal_confirm_duration': 0.3,
                'stopline_pause_duration': 3.2,
            }],
            output='screen'
        )
    ])
```

---

## 🔄 모드 전환 로직

### 예선 모드 전환

```
DRIVING (기본)
  ├─ 주차 구역 진입 → PARKING
  │   └─ 주차 완료 → DRIVING
  ├─ 유턴 구역 진입 → UTURN
  │   └─ 구역 이탈 → DRIVING
  └─ GPS 차단 구역 진입 → GPS_OFF
      └─ 구역 이탈 → DRIVING
```

### 본선 모드 전환

```
DRIVING (기본)
  ├─ Red 신호등 → PAUSE
  │   └─ Green/Left/Straightleft (0.3초) → DRIVING
  │
  ├─ 우회전 정지선 구역 (9,10)
  │   └─ right stopline → PAUSE
  │       └─ 정지 + 3.2초 → DRIVING
  │
  ├─ 주차 구역 + 진입 → PARKING
  │   └─ 주차 완료 → DRIVING
  │
  └─ 배달 미션
      ├─ 상차 표지판 (1-3) → DELIVERY_PICKUP
      │   └─ 상차 완료 → DRIVING
      └─ 하차 구역 + 하차 표지판 (4-6) → DELIVERY_DROPOFF
          └─ 하차 완료 → DRIVING
```

---

## 🛠️ 개발 정보

### 버전
- **v2.0**: 예선/본선 분리, 신호등 타이머, 배달 상차/하차 분리
- **v1.0**: 초기 버전

### 의존성
- `rclpy`
- `std_msgs`
- `nav_msgs`
- `planning_msgs`

### 디렉토리 구조
```
mode/
├── mode/
│   ├── __init__.py
│   └── selector.py      # 메인 노드
├── resource/
│   └── mode
├── setup.py
├── setup.cfg
├── package.xml
└── README.md
```

---

## 📝 변경 이력

### 2025-10-18
- 신호등 전환 0.3초 확인 타이머 추가
- 배달 미션 상차/하차 플래그 분리 (`pickup_complete_flag` 추가)
- 상차 완료 후 DRIVING 모드 복귀 로직 추가
- 문서 업데이트

### 이전
- 예선/본선 모드 분리
- K-City/미래관 맵 지원
- 우회전 정지선 특수 처리
- Lanelet ID 직접 사용 (Map ID 변환 제거)

---

## 🤝 기여

버그 리포트나 기능 제안은 팀 내부 채널을 통해 공유해주세요.

---

## 📄 라이선스

TODO

---

## 👥 관리자

- Maintainer: user
- Email: user@todo.todo

