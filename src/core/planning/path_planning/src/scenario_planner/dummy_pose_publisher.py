import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class DummyPosePublisher(Node):
    def __init__(self):
        super().__init__('dummy_pose_publisher')
        # QoS 설정: 라칭을 위해 Transient Local
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle_pose', qos)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # 초기 포즈 설정
        self.init_pose = PoseStamped()
        self.init_pose.header.frame_id = 'map'
        self.init_pose.pose.position.x = 0.0
        self.init_pose.pose.position.y = 0.0
        self.init_pose.pose.position.z = 0.0
        self.init_pose.pose.orientation.w = 1.0

        self.current_path = []
        self.index = 0

    def path_callback(self, msg: Path):
        # 새로운 경로를 수신하면 저장
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.index = 0
        self.get_logger().info(f"📥 경로 수신: {len(self.current_path)} 포인트")

    def timer_callback(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        if self.current_path and self.index < len(self.current_path):
            x, y = self.current_path[self.index]
            self.index += 1
        else:
            # 경로가 없으면 초기 포즈를 유지
            x = self.init_pose.pose.position.x
            y = self.init_pose.pose.position.y

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
