import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
import numpy as np

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        self.car_pos = None
        self.global_path = []
        self.obstacles = []
        self.stopline = None
        self.local_paths = {
            'drive': [],
            'pause': [],
            'static_obstacle': [],
            'delivery': []
        }
        self.delivery_zone = []
        self.unload_zone = []
        self.parking_zone = []
        self.stopline_points = []
        self.mission_state = ''
        self.colors = {
            'car': 'black',
            'global_path': 'gray',
            'obstacle': 'red',
            'stopline': 'orange',
            'delivery_zone': 'green',
            'unload_zone': 'blue',
            'parking_zone': 'purple',
            'local_drive': 'red',
            'local_pause': 'orange',
            'local_static_obstacle': 'cyan',
            'local_delivery': 'magenta',
        }
        self.zone_alpha = 0.3
        # Plot limits based on global path
        self.plot_xlim = None
        self.plot_ylim = None
        self.plot_limits_set = False
        self.create_subscription(Odometry, '/localization', self.odom_callback, 10)
        self.create_subscription(Path, '/global_waypoints', self.global_path_callback, 10)
        self.create_subscription(PointCloud2, '/obstacle_detector', self.obstacle_callback, 10)
        self.create_subscription(PoseArray, '/local_path', self.local_path_callback, 10)
        self.create_subscription(PoseArray, '/delivery_zone', self.delivery_zone_callback, 10)
        self.create_subscription(PoseArray, '/unload_zone', self.unload_zone_callback, 10)
        self.create_subscription(PoseArray, '/parking_zone', self.parking_zone_callback, 10)
        self.create_subscription(PoseArray, '/stopline_array', self.stopline_array_callback, 10)
        self.create_subscription(String, '/mission_state', self.mission_state_callback, 10)
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.2, self.update_plot)
        plt.ion()
        plt.show()

    def odom_callback(self, msg):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def global_path_callback(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.global_path and not self.plot_limits_set:
            xs, ys = zip(*self.global_path)
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            # Add a buffer around the global path extent
            buffer = 10.0
            self.plot_xlim = (min_x - buffer, max_x + buffer)
            self.plot_ylim = (min_y - buffer, max_y + buffer)
            self.plot_limits_set = True
            self.get_logger().info("Calculated and stored plot limits based on global path.")

    def obstacle_callback(self, msg):
        self.obstacles = [(x, y) for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]

    def local_path_callback(self, msg):
        kind = msg.header.frame_id if msg.header.frame_id in self.local_paths else 'drive'
        self.local_paths[kind] = [(p.position.x, p.position.y) for p in msg.poses]

    def delivery_zone_callback(self, msg):
        self.delivery_zone = [(p.position.x, p.position.y) for p in msg.poses]

    def unload_zone_callback(self, msg):
        self.unload_zone = [(p.position.x, p.position.y) for p in msg.poses]

    def parking_zone_callback(self, msg):
        self.parking_zone = [(p.position.x, p.position.y) for p in msg.poses]

    def stopline_array_callback(self, msg):
        self.stopline_points = [(p.position.x, p.position.y) for p in msg.poses]

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

    def update_plot(self):
        self.ax.clear()

        # Re-apply plot limits if set
        if self.plot_limits_set:
            self.ax.set_xlim(self.plot_xlim)
            self.ax.set_ylim(self.plot_ylim)
            self.ax.set_aspect('equal', adjustable='box')

        if self.global_path:
            xs, ys = zip(*self.global_path)
            self.ax.plot(xs, ys, color=self.colors['global_path'], label='Global Path')
        for kind, path in self.local_paths.items():
            if path:
                xs, ys = zip(*path)
                self.ax.plot(xs, ys, color=self.colors.get(f'local_{kind}', 'red'), linestyle='--', label=f'Local {kind}')
        if self.car_pos:
            self.ax.plot(self.car_pos[0], self.car_pos[1], 'o', color=self.colors['car'], label='Car')
        if self.obstacles:
            xs, ys = zip(*self.obstacles)
            self.ax.scatter(xs, ys, color=self.colors['obstacle'], marker='x', label='Obstacles')
        if self.stopline_points:
            xs, ys = zip(*self.stopline_points)
            self.ax.scatter(xs, ys, color=self.colors['stopline'], marker='s', label='Stoplines')
        if self.delivery_zone:
            poly = plt.Polygon(self.delivery_zone, color=self.colors['delivery_zone'], alpha=self.zone_alpha, label='Delivery Zone')
            self.ax.add_patch(poly)
        if self.unload_zone:
            poly = plt.Polygon(self.unload_zone, color=self.colors['unload_zone'], alpha=self.zone_alpha, label='Unload Zone')
            self.ax.add_patch(poly)
        if self.parking_zone:
            poly = plt.Polygon(self.parking_zone, color=self.colors['parking_zone'], alpha=self.zone_alpha, label='Parking Zone')
            self.ax.add_patch(poly)
        self.ax.text(0.01, 0.99, f'Mission: {self.mission_state}', transform=self.ax.transAxes, fontsize=12, verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))
        self.ax.legend(loc='upper right')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Path Planning Visualization')
        self.ax.grid(True)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 