import numpy as np
import matplotlib.pyplot as plt
from hybrid_astar_sha import ScenarioHybridAStar

# 장애물 좌표 (예시: 없음)
ox, oy = [], []

# 파라미터
resolution = 1.0
wheel_base = 2.5
scenario_weights = {
    'drive': {'forward': 1.0, 'backward': 2.0}
}

# Hybrid A* 객체 생성
planner = ScenarioHybridAStar(ox, oy, resolution, wheel_base, scenario_weights)

# 시작점(x, y, yaw), 목표점(x, y, yaw)
start = (0.0, 0.0, 0.0)
goal  = (10.0, 10.0, 0.0)

# 경로 탐색
path = planner.planning(start, goal, scenario='drive')

# 결과 출력 및 시각화
if path is not None:
    path = np.array(path)
    plt.plot(path[:,0], path[:,1], '-o', label='Hybrid A* path')
    plt.plot(start[0], start[1], 'go', label='Start')
    plt.plot(goal[0], goal[1], 'ro', label='Goal')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()
else:
    print("경로를 찾지 못했습니다.")