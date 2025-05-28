import numpy as np
from scipy.spatial import KDTree

class WaySelector():
    def __init__(self, ways, way_nodes):
        self.ways = ways
        self.way_nodes = way_nodes

        self.ways_kdtree = {}
        print(self.ways.values())
        print("=============")
        print(self.way_nodes.keys())
        for way_id in self.ways:
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
            self.ways_kdtree[way_id] = KDTree(waypoints)

        self.clicked_point_list = []
        self.selected_ways = []
        self.selected_ways_list = []

    def update_click_input(self, click_input):
        # 클릭한 좌표를 clicked_point_list에 추가
        self.clicked_point_list.append(click_input)

        # clicked_way 찾기
        clicked_way = self.find_closest_way(click_input)

        # candidate_ways 찾기 (clicked_way 기준)
        candidate_ways = self.find_candidate_ways(clicked_way)

        # 지금까지 선택된 경로 재계산하기
        selected_ways = self.update_selected_ways()

        return clicked_way, candidate_ways, selected_ways
    
    def find_closest_way(self, cur_position):
        """ 현재 위치로부터 가장 가까운 way 찾기 """
        # cur_position: clicked_point (x,y)
        cur_position_list = [cur_position.point.x, cur_position.point.y]

        closest_way = None
        min_distance = np.inf

        for way_id, tree in self.ways_kdtree.items():
            distance, _ = tree.query(cur_position_list)
            
            if distance < min_distance:
                min_distance = distance
                closest_way = way_id


        return closest_way
    
    def find_candidate_ways(self, cur_way_id):
        candidate_ways = []
        cur_way = self.ways[cur_way_id]
        
        for way_id in self.ways:
            if (way_id == cur_way_id):
                continue
            
            # 첫 번째 조건: 현재 way의 마지막 노드와 특정 way의 처음 노드가 일정 거리 이내일 때
            way = self.ways[way_id]

            next_way_start_node = self.way_nodes[way[0]]
            cur_way_end_node = self.way_nodes[cur_way[-1]]
            dist = np.sqrt((next_way_start_node[0]-cur_way_end_node[0])**2 + (next_way_start_node[1]-cur_way_end_node[1])**2)

            if dist > 10:
                continue

            # 두 번째 조건: 현재 way의 마지막 부분 기울기와 특정 way의 처음 부분 기울기가 같은 부호일 때 or 비율로 ~% 이상일 때
            cur_way_direction = np.rad2deg(np.arctan2(self.way_nodes[cur_way[-1]][1]-self.way_nodes[cur_way[-2]][1], self.way_nodes[cur_way[-1]][0]-self.way_nodes[cur_way[-2]][0]))
            next_way_direction = np.rad2deg(np.arctan2(self.way_nodes[way[1]][1]-self.way_nodes[way[0]][1], self.way_nodes[way[1]][0]-self.way_nodes[way[0]][0]))

            direction_similarity = abs((cur_way_direction - next_way_direction))
            if direction_similarity < 50 or direction_similarity > 310:
                candidate_ways.append(way_id)

        return candidate_ways
    
    def update_selected_ways(self):
        target_ways = []
        target_point = self.clicked_point_list[-1]

        if len(self.clicked_point_list) == 1:
            first_way = self.find_closest_way(self.clicked_point_list[0])
            target_ways.append(first_way)

        else:    
            start_way = self.find_closest_way(self.clicked_point_list[-2])
            last_way = self.find_closest_way(self.clicked_point_list[-1])
        
            i = 0
            cur_way = start_way
            while True:
                if(cur_way == last_way):
                    break
                
                # print(cur_way)
                candidate_ways = self.find_candidate_ways(cur_way)
                cur_way = self.choose_candidate_way(candidate_ways, target_point)
                target_ways.append(cur_way)

                i += 1
                if(i>7):
                    break
        
        self.selected_ways += target_ways
        self.selected_ways_list.append(target_ways)
        
        return target_ways

    def remove_target_ways(self):
        if len(self.clicked_point_list) == 0:
            return
        
        for way in self.selected_ways_list[-1]:
            self.selected_ways.remove(way)
        self.selected_ways_list.pop()
        self.clicked_point_list.pop()
        return
    
    def reset_selected_ways(self):
        self.clicked_point_list = []
        self.selected_ways = []
        return
    
    def choose_candidate_way(self, candidate_ways, target_point):
        cost_prev = np.inf
        next_way = None

        for candidate_way_id in candidate_ways:
            candidate_way = self.ways[candidate_way_id]
            cost = (self.way_nodes[candidate_way[-1]][0] - target_point.point.x)**2 + \
                (self.way_nodes[candidate_way[-1]][1] - target_point.point.y)**2
            if cost < cost_prev:
                next_way = candidate_way_id
                cost_prev = cost

        return next_way