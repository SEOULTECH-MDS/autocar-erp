# 전체 콘 맵 메시지
std_msgs/Header header

# 좌표계 정보
bool coordinate_system_ready          # 정규화된 좌표계 준비 여부
geometry_msgs/Point origin            # 좌표계 원점 (map frame)
float32 rotation_angle               # 회전 각도 (radians)

# 콘 데이터
ConeInfo[] cones                     # 저장된 모든 콘

# 메타데이터
uint32 total_cone_count              # 총 콘 개수
uint32 reliable_cone_count           # 신뢰도 높은 콘 개수 (confidence >= 2.0)
builtin_interfaces/Time last_update  # 마지막 업데이트 시간
