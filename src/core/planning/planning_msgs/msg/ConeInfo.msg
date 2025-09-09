# 개별 콘 정보
uint32 id
geometry_msgs/Point position          # 정규화된 좌표계에서의 위치
geometry_msgs/Point original_position # 원본 map frame 위치
string side_label                     # "left", "right", "center", "unknown"
string area_label                     # "open_area_1", "closed_area_2", "unknown" (구역 판단 후 설정)
float32 confidence                    # 신뢰도 (관측 횟수 기반)
uint32 observation_count              # 관측 횟수
builtin_interfaces/Time last_seen     # 마지막 관측 시간
