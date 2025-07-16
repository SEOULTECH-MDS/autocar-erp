import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
import sys, os
import numpy as np

# 로컬 hybrid_astar 모듈 사용
try:
    from .hybrid_astar.hybrid_a_star import hybrid_a_star_planning
    use_real_planner = True
except ImportError:
    use_real_planner = False
    print("Warning: Local hybrid_astar module not found, using fallback planner")

class HybridAStarPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        self.current_pose = None
        self.goal = None
        self.mode = 'normal'

        # 구독자
        self.create_subscription(String, '/scenario_mode', self.mode_callback, 10)
        self.create_subscription(PoseStamped, '/vehicle_pose', self.pose_callback, 10)
        # 발행자
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info('HybridAStarPlanner 노드 초기화 완료')

    def mode_callback(self, msg):
        self.mode = msg.data
        self.get_logger().info(f'모드 변경 수신: {self.mode}')
        if self.mode == 'delivery':
            self.goal = (10.0, 5.0, 0.0)
        elif self.mode == 'parking':
            self.goal = (15.0, 10.0, 0.0)
        elif self.mode == 'return':
            self.get_logger().info('return 모드: 골 설정 필요')
            return
        else:
            self.goal = None
        # goal이 설정될 때마다 경로 계획 실행
        if self.goal:
            self.plan_path()

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = 0.0
        self.current_pose = (x, y, yaw)
        self.get_logger().debug(f'현재 위치 수신: x={x:.2f}, y={y:.2f}')
        # goal이 이미 설정되어 있으면 재계획
        if self.goal:
            self.plan_path()

    def plan_path(self):
        if self.current_pose is None or self.goal is None:
            self.get_logger().warn('경로 계획 불가: pose 또는 goal 미설정')
            return

        sx, sy, syaw = self.current_pose
        gx, gy, gyaw = self.goal
        self.get_logger().info(f'🚀 경로 계획: start=({sx:.2f},{sy:.2f}), goal=({gx:.2f},{gy:.2f})')

        rx, ry, ryaw = [], [], []

        if use_real_planner:
            try:
                # Hybrid A* 알고리즘 호출
                rx, ry, ryaw, _, _ = hybrid_a_star_planning(
                    sx=sx, sy=sy, syaw=syaw,
                    gx=gx, gy=gy, gyaw=gyaw,
                    grid_size=1.0, robot_radius=1.0
                )
            except Exception as e:
                self.get_logger().warn(f'하이브리드 A* 실패({e}), fallback 사용')

        # fallback 직선 경로
        if not rx:
            dist = np.hypot(gx - sx, gy - sy)
            pts = max(int(dist / 0.5), 2)
            xs = np.linspace(sx, gx, pts)
            ys = np.linspace(sy, gy, pts)
            rx, ry, ryaw = xs.tolist(), ys.tolist(), [0.0] * pts

        path = Path()
        path.header.frame_id = 'map'
        for x, y, yaw in zip(rx, ry, ryaw):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            path.poses.append(ps)

        self.path_pub.publish(path)
        self.get_logger().info(f'✅ 경로 발행: {len(rx)} 포인트')



def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
