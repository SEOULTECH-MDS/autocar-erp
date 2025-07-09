import numpy as np
from . import dubins_path_planner as dubins
from heapdict import heapdict

class Node:
    def __init__(self, x, y, yaw, cost, parent=None):
        self.x, self.y, self.yaw = x, y, yaw
        self.cost = cost      # g-value
        self.parent = parent

    def state(self):
        return (self.x, self.y, self.yaw)

def heuristic(n1, n2):
    # 유클리드 거리
    dx = n2.x - n1.x
    dy = n2.y - n1.y
    return np.hypot(dx, dy)

class ScenarioHybridAStar:
    def __init__(self, ox, oy, resolution, wheel_base, scenario_weights):
        """
        ox, oy: 장애물 좌표 리스트
        resolution: 격자 해상도
        wheel_base: 차량 휠베이스 (곡률 계산용)
        scenario_weights: {'parking': {'forward':1.0,'backward':2.0}, ...}
        """
        self.ox, self.oy = ox, oy
        self.reso = resolution
        self.wb = wheel_base
        self.sc_w = scenario_weights

    def planning(self, start, goal, scenario='drive'):
        openset = heapdict()
        closed = dict()

        start_node = Node(*start, cost=0.0)
        goal_node  = Node(*goal,  cost=0.0)
        openset[start_node.state()] = self.cost_func(start_node, goal_node, scenario)

        while openset:
            cur_state, _ = openset.popitem()
            cnode = closed[cur_state] if cur_state in closed else Node(*cur_state, cost=0.0)
            if heuristic(cnode, goal_node) < self.reso:
                return self._reconstruct_path(cnode)

            closed[cur_state] = cnode

            for nxt in self._expand(cnode, scenario):
                st = nxt.state()
                new_cost = cnode.cost + nxt.cost
                if st not in closed or closed[st].cost > new_cost:
                    nxt.cost = new_cost
                    closed[st] = nxt
                    f = new_cost + heuristic(nxt, goal_node)
                    openset[st] = f

        return None  # 실패

    def cost_func(self, node, goal, scenario):
        return node.cost + heuristic(node, goal)

    def _expand(self, node, scenario):
        motions = self._get_motion_primitives(node, scenario)
        result = []
        for steer, direction in motions:
            x_list, y_list, yaw_list, mode, clen = dubins.plan_dubins_path(
                node.x, node.y, node.yaw,
                node.x + np.cos(node.yaw), node.y + np.sin(node.yaw), node.yaw + steer,
                1.0 / self.wb,  # curvature = 1/radius
                step_size=self.reso
            )
            x2, y2, yaw2 = x_list[-1], y_list[-1], yaw_list[-1]
            # 시나리오별 전진/후진 비용 적용
            w = self.sc_w[scenario]['forward'] if direction == 1 else self.sc_w[scenario]['backward']
            cost = direction * self.reso * w
            result.append(Node(x2, y2, yaw2, cost, parent=node))
        return result

    def _get_motion_primitives(self, node, scenario):
        # (steering_angle, direction) 리스트 반환
        # 예: 전진 3가지 조향, 후진 1가지 조향
        fw = [(np.deg2rad(a),  1) for a in [-15, 0, 15]]
        bw = [(np.deg2rad(a), -1) for a in [0]]
        return fw + bw

    def _reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y, node.yaw))
            node = node.parent
        return path[::-1]
