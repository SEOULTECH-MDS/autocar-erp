#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


class RoadNode(Node):
    """
    더미 인식 노드: 도로 양쪽 차선을 두 개의 선분으로 Rviz에 시각화하여 퍼블리시

    - 좌측 차선: (0, 0) -> (0, 30)
    - 우측 차선: (3, 0) -> (3, 30)
    """

    def __init__(self) -> None:
        super().__init__('road_node')

        # 퍼블리셔: Rviz용 마커
        self.marker_pub = self.create_publisher(MarkerArray, '/road_markers', 10)

        # 마커 사전 구성 (타이머에서 타임스탬프만 갱신)
        self.markers = self._create_lane_markers()

        # 10Hz 주기로 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.get_logger().info('Road node 시작: 좌/우 차선 마커를 /road_markers 로 퍼블리시합니다.')

    def _create_lane_markers(self) -> MarkerArray:
        marker_array = MarkerArray()

        # 공통 설정 값
        line_width = 0.08

        # 좌측 차선 (노란색)
        left_marker = self._make_line_marker(
            marker_id=0,
            ns='lane_left',
            points=[(0.0, 0.0), (0.0, 30.0)],
            color=(1.0, 1.0, 0.0, 1.0),
            line_width=line_width,
        )
        marker_array.markers.append(left_marker)

        # 우측 차선 (흰색)
        right_marker = self._make_line_marker(
            marker_id=1,
            ns='lane_right',
            points=[(3.0, 0.0), (3.0, 30.0)],
            color=(1.0, 1.0, 1.0, 1.0),
            line_width=line_width,
        )
        marker_array.markers.append(right_marker)

        return marker_array

    def _make_line_marker(self, marker_id: int, ns: str, points, color, line_width: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # 위치(기본값) 및 자세
        marker.pose.orientation.w = 1.0

        # 선 두께
        marker.scale = Vector3()
        marker.scale.x = line_width

        # 색상
        marker.color = ColorRGBA()
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        # 점 추가
        for x, y in points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            marker.points.append(p)

        return marker

    def publish_markers(self) -> None:
        # 타임스탬프 갱신 후 퍼블리시
        now = self.get_clock().now().to_msg()
        for mk in self.markers.markers:
            mk.header.stamp = now
        self.marker_pub.publish(self.markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


