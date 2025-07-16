import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point as ROSPoint
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/vehicle_pose', self.pose_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(String, '/scenario_mode', self.mode_callback, 10)

        # Goal state
        self.goal = None

    def mode_callback(self, msg: String):
        # Update goal based on scenario mode
        if msg.data == 'delivery':
            self.goal = (10.0, 5.0)
        elif msg.data == 'parking':
            self.goal = (15.0, 10.0)
        else:
            self.goal = None
        self.publish_goal_marker()

    def publish_goal_marker(self):
        # Publish a sphere marker at the goal
        if self.goal is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal[0]
        marker.pose.position.y = self.goal[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def pose_callback(self, msg: PoseStamped):
        # Publish an arrow marker for the vehicle's current pose
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'vehicle'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 1.0  # arrow length
        marker.scale.y = 0.2  # arrow width
        marker.scale.z = 0.2  # arrow height
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def path_callback(self, msg: Path):
        # Publish a line strip marker for the planned path
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'planned_path'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        for pose_stamped in msg.poses:
            pt = ROSPoint()
            pt.x = pose_stamped.pose.position.x
            pt.y = pose_stamped.pose.position.y
            pt.z = 0.0
            marker.points.append(pt)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()