import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class RegionFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('region_filter_node')

        self.input_topic = self.declare_parameter('input_topic', '/velodyne_points').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/velodyne_points/filtered').get_parameter_value().string_value

        # 제거할 영역 파라미터 (박스 기준)
        # min/max_x, y, z 로 정의. 기본값은 비활성(-inf, +inf)
        self.min_x = float(self.declare_parameter('min_x', float('-inf')).get_parameter_value().double_value)
        self.max_x = float(self.declare_parameter('max_x', float('inf')).get_parameter_value().double_value)
        self.min_y = float(self.declare_parameter('min_y', float('-inf')).get_parameter_value().double_value)
        self.max_y = float(self.declare_parameter('max_y', float('inf')).get_parameter_value().double_value)
        self.min_z = float(self.declare_parameter('min_z', float('-inf')).get_parameter_value().double_value)
        self.max_z = float(self.declare_parameter('max_z', float('inf')).get_parameter_value().double_value)

        # inside_mode: True 이면 박스 내부만 유지, False 이면 내부 제거(=외부만 유지)
        self.inside_mode = self.declare_parameter('inside_mode', False).get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'RegionFilterNode started. input={self.input_topic}, output={self.output_topic}, '
            f'box=([{self.min_x}, {self.max_x}], [{self.min_y}, {self.max_y}], [{self.min_z}, {self.max_z}]), '
            f'inside_mode={self.inside_mode}'
        )

    def _point_in_box(self, x: float, y: float, z: float) -> bool:
        if x < self.min_x or x > self.max_x:
            return False
        if y < self.min_y or y > self.max_y:
            return False
        if z < self.min_z or z > self.max_z:
            return False
        return True

    def _on_cloud(self, msg: PointCloud2) -> None:
        header = msg.header
        fields = msg.fields
        is_bigendian = msg.is_bigendian
        is_dense = msg.is_dense

        # 원본 필드 순서를 그대로 보존
        field_names = [f.name for f in fields]
        try:
            ix = field_names.index('x')
            iy = field_names.index('y')
            iz = field_names.index('z')
        except ValueError:
            self.get_logger().error('Incoming PointCloud2 does not contain x/y/z fields')
            return

        filtered_points = []
        for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            x = float(point[ix])
            y = float(point[iy])
            z = float(point[iz])
            inside = self._point_in_box(x, y, z)
            keep = inside if self.inside_mode else not inside
            if keep:
                filtered_points.append(point)

        # 원본 필드 레이아웃을 유지한 채로 퍼블리시
        out = pc2.create_cloud(header, fields, filtered_points)
        out.is_bigendian = is_bigendian
        out.is_dense = is_dense
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = RegionFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


