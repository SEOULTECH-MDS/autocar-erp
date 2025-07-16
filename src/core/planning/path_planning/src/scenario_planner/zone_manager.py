import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from shapely.geometry import Point, Polygon
from visualization_msgs.msg import Marker

class ZoneManager(Node):
    def __init__(self):
        super().__init__('zone_manager')

        # 1. Zone 정의
        self.zones = {
            'delivery': {
                'polygon': Polygon([(9, 4), (11, 4), (11, 6), (9, 6)]),
                'color': (1.0, 0.0, 0.0, 0.6)  # 빨간색
            },
            'parking': {
                'polygon': Polygon([(14, 9), (16, 9), (16, 11), (14, 11)]),
                'color': (0.0, 0.0, 1.0, 0.6)  # 파란색
            }
        }

        # 2. 퍼블리셔/서브스크라이버 설정
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self.pose_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/zone_markers', 10)

        # 3. 시작 시 Marker 발행
        self.publish_markers()

    def pose_callback(self, msg):
        current = Point(msg.pose.position.x, msg.pose.position.y)
        for name, zone in self.zones.items():
            if zone['polygon'].contains(current):
                self.get_logger().info(f"✅ 차량이 {name.upper()} 구역에 진입했습니다.")

    def publish_markers(self):
        from geometry_msgs.msg import Point as PointMsg
        for idx, (name, zone) in enumerate(self.zones.items()):
            poly = zone['polygon']
            color = zone['color']

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.ns = name
            marker.id = idx
            marker.scale.x = 0.1
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

            for x, y in list(poly.exterior.coords):
                pt = PointMsg()
                pt.x = x
                pt.y = y
                pt.z = 0.0
                marker.points.append(pt)

            # 폴리곤 닫기 (시작점으로 돌아가기)
            x0, y0 = poly.exterior.coords[0]
            pt0 = PointMsg()
            pt0.x = x0
            pt0.y = y0
            pt0.z = 0.0
            marker.points.append(pt0)

            self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ZoneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
