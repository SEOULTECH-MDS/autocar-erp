#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Header, String
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from autocar_utils.euler_from_quaternion import euler_from_quaternion
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
from autocar_utils.normalise_angle import normalise_angle
from .acados_setting_sp import acados_solver
from autocar_utils.utils import CubicSpline2D

from rviz_2d_overlay_msgs.msg import OverlayText
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ModeState
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy


# State machine for traffic-light aware stopping/going
STATE_APPROACH = 0
STATE_DECELERATE = 1
STATE_STOPPED = 2
STATE_GO = 3

NX = 5  # (x, y, yaw, v, s)
NU = 2  # (delta, a)
T = 3.0
N = 30


class Control(Node):
    def __init__(self):
        super().__init__('control_traffic')

        # Parameters (could be declared as ROS2 params later)
        self.approach_distance_threshold = 20.0 # 이 거리 이내에서만 신호 준수 로직 활성
        self.stopline_margin = 0.8 # 차량이 정지선에 도달하기 전에 미리 멈추는 거리
        self.min_hold_time_at_red = 1.0 # 빨간색 신호 유지 최소 시간 (s)
        self.green_confirm_time = 0.7 # 녹색 신호 확인 윈도우 (s)
        self.yellow_treat_as_red = False # 노란색도 정지 신호로 처리
        self.v_stop_threshold = 0.3 # 차량이 정지할 속도 임계값 (m/s)
        self.clear_distance = 5.0 # 정지선을 지나치기 전에 차량이 정지할 거리 (m)
        self.max_decel_comfort = 10.0 # 차량이 정지할 때 최대 감속량 (m/s^2)
        self.tl_confirm_window = 0.5 # 신호 상태 확인 윈도우 (s)

        # Publisher
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        # Visualization publishers
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/overlay', 10)
        self.global_path_pub = self.create_publisher(MarkerArray, '/autocar/global_path', qos_profile=qos_transient_local)
        self.mpc_predict_pub = self.create_publisher(MarkerArray, '/autocar/mpc_predict', qos_profile=qos_transient_local)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', qos_profile=qos_transient_local)

        # Subscribers
        self.localization_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/autocar/goals', self.global_waypoints_cb, 10)
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_cb, 10)
        self.local_waypoints_sub = self.create_subscription(Path, '/waypoints', self.local_waypoints_cb, 10)
        self.map_origin_sub = self.create_subscription(PointStamped, '/map/origin', self.map_origin_cb, qos_transient_local)
        self.obstacle_sub = self.create_subscription(MarkerArray, '/obstacles/markers', self.obstacle_cb, 10)
        # Traffic light and localization-provided stopline distance
        self.tl_sub = self.create_subscription(String, '/traffic_sign', self.traffic_cb, 10)
        self.stopline_dist_sub = self.create_subscription(Float64, '/stopline_distance', self.stopline_dist_cb, 10)
        self.stopline_type_sub = self.create_subscription(String, '/stopline_type', self.stopline_type_cb, 10)

        # Vehicle state
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None
        self.s = None

        # Paths
        self.xs_global = []
        self.ys_global = []
        self.cubic_spline_global = None

        self.xs_local = []
        self.ys_local = []
        self.cubic_spline_local = None

        self.lock = threading.Lock()
        self.control_frequency = 20.0
        self.dt = T / N

        # Obstacles
        self.obs1_x = None
        self.obs1_y = None
        self.obs2_x = None
        self.obs2_y = None

        # Stopline distance handling
        self.stopline_distance = float('inf')
        self.stopline_type = 'no_stopline'

        # Control outputs
        self.target_vel = 4.0
        self.steering_angle = 0.0
        self.velocity = 0.0
        self.prev_steering_angle = 0.0
        self.prev_velocity = 0.0
        self.fail_count = 0

        # s monotonicity
        self.prev_s = 0.0
        self.s_tolerance = 30.0

        # Map origin
        self.map_origin_x = None
        self.map_origin_y = None

        # Mode
        self.mode = 0
        self.mode_description = "Drive"

        # Traffic light state and FSM
        self.tl_state_raw = 'None'
        self.tl_state_confirmed = 'None'
        self.tl_last_seen = None
        self.tl_current_raw_since = None
        self.fsm_state = STATE_APPROACH
        self.last_stop_time = None

        # MPC Solver
        self.solver = acados_solver()

        # Timer
        self.timer_control = self.create_timer(1.0 / self.control_frequency, self.mpc_control)

    def map_origin_cb(self, msg):
        self.map_origin_x = msg.point.x
        self.map_origin_y = msg.point.y

    def vehicle_state_cb(self, msg):
        if self.map_origin_x is None or self.map_origin_y is None:
            self.get_logger().warn("Map origin 정보가 설정되지 않았습니다. 차량의 상태를 업데이트할 수 없습니다.")
            return

        self.lock.acquire()

        self.x = msg.pose.pose.position.x - self.map_origin_x
        self.y = msg.pose.pose.position.y - self.map_origin_y

        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.v = np.sqrt((msg.twist.twist.linear.x ** 2.0) + (msg.twist.twist.linear.y ** 2.0))
        if abs(self.v) < 0.001:
            self.v = 0.1
        self.yawrate = msg.twist.twist.angular.z

        self.lock.release()

    def mode_cb(self, msg):
        self.mode = msg.current_mode
        self.mode_description = msg.description

    def calc_current_s(self, _cubic_spline):
        cubic_spline = _cubic_spline
        if self.x is None or self.y is None:
            self.s = 0.0
            return
        if not hasattr(cubic_spline, 's') or len(cubic_spline.s) == 0:
            self.s = 0.0
            return

        total_length = cubic_spline.s[-1]
        search_start = max(0, self.prev_s - self.s_tolerance)
        search_end = min(total_length, self.prev_s + 50.0)

        coarse_resolution = 1.0
        s_coarse = np.arange(search_start, search_end + coarse_resolution, coarse_resolution)

        min_distance = float('inf')
        best_s = self.prev_s
        for s_val in s_coarse:
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val

        search_range = 2.0
        s_start = max(search_start, best_s - search_range)
        s_end = min(search_end, best_s + search_range)
        fine_resolution = 0.05
        s_fine = np.arange(s_start, s_end + fine_resolution, fine_resolution)
        for s_val in s_fine:
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val

        if best_s < self.prev_s - self.s_tolerance:
            best_s = self.prev_s - self.s_tolerance
            self.get_logger().warn(f"S 값 역행 제한: {best_s:.2f} (이전: {self.prev_s:.2f})")

        self.prev_s = best_s
        self.s = best_s
        return

    def global_waypoints_cb(self, path_msg):
        self.xs_global, self.ys_global = [], []
        for node in path_msg.poses:
            self.xs_global.append(node.position.x)
            self.ys_global.append(node.position.y)
        self.make_cubic_spline()
        return

    def local_waypoints_cb(self, path_msg):
        self.xs_local, self.ys_local = [], []
        if self.map_origin_x is not None and self.map_origin_y is not None:
            for node in path_msg.poses:
                self.xs_local.append(node.pose.position.x - self.map_origin_x)
                self.ys_local.append(node.pose.position.y - self.map_origin_y)
        else:
            self.get_logger().warn("Map origin 정보가 설정되지 않았습니다. Local waypoints를 업데이트할 수 없습니다.")
            return
        self.make_cubic_spline()
        return

    def make_cubic_spline(self):
        if len(self.xs_global) >= 2 and len(self.ys_global) >= 2:
            self.cubic_spline_global = CubicSpline2D(self.xs_global, self.ys_global)
            self.get_logger().info(f"Global cubic spline 생성 완료: {len(self.xs_global)}개 포인트")
        if len(self.xs_local) >= 2 and len(self.ys_local) >= 2:
            self.cubic_spline_local = CubicSpline2D(self.xs_local, self.ys_local)
            self.get_logger().info(f"Local cubic spline 생성 완료: {len(self.xs_local)}개 포인트")

    def obstacle_cb(self, msg):
        self.obs1_x = msg.markers[0].pose.position.x - self.map_origin_x
        self.obs1_y = msg.markers[0].pose.position.y - self.map_origin_y
        self.obs2_x = msg.markers[1].pose.position.x - self.map_origin_x
        self.obs2_y = msg.markers[1].pose.position.y - self.map_origin_y

    def traffic_cb(self, msg: String):
        now = self.get_clock().now()
        raw = msg.data if msg and isinstance(msg.data, str) else 'None'
        self.tl_state_raw = raw
        if self.tl_current_raw_since is None or raw != self.tl_state_confirmed:
            # If raw differs from confirmed, start (or reset) stability timer
            self.tl_current_raw_since = now
        # Confirm if raw is stable for tl_confirm_window
        try:
            elapsed = (now - self.tl_current_raw_since).nanoseconds * 1e-9 if self.tl_current_raw_since else 0.0
        except Exception:
            elapsed = 0.0
        if elapsed >= self.tl_confirm_window:
            self.tl_state_confirmed = raw
            self.tl_last_seen = now

    def stopline_dist_cb(self, msg: Float64):
        try:
            self.stopline_distance = float(msg.data)
        except Exception:
            self.stopline_distance = float('inf')

    def stopline_type_cb(self, msg: String):
        try:
            self.stopline_type = str(msg.data).strip()
        except Exception:
            self.stopline_type = 'no_stopline'

    def calc_ref_trajectory(self, _cubic_spline):
        cubic_spline = _cubic_spline
        xref = np.zeros((NX, N))
        tan_vec = np.zeros((2, N))
        if cubic_spline:
            current_s = self.s
            for i in range(N):
                s = current_s + self.dt * self.target_vel
                if s > cubic_spline.s[-1]:
                    s = cubic_spline.s[-1] - 0.5
                    target_vel = 0.0
                else:
                    remaining_distance = cubic_spline.s[-1] - s
                    if remaining_distance <= 3.0:
                        target_vel = 0.0
                    else:
                        target_vel = self.target_vel
                current_s = s
                xref[0, i], xref[1, i] = cubic_spline.calc_position(s)
                xref[2, i] = cubic_spline.calc_yaw(s)
                xref[3, i] = target_vel
                xref[4, i] = s
                tan_vec[0, i] = math.cos(cubic_spline.calc_yaw(s))
                tan_vec[1, i] = math.sin(cubic_spline.calc_yaw(s))
        self.visualize_ref_trajectory(xref)
        return xref, tan_vec

    def _get_effective_stopline_distance(self) -> float:
        # Ignore stoplines when type is not traffic-signal related
        if not self._is_stopline_relevant():
            return float('inf')
        d = self.stopline_distance
        if d is None:
            return float('inf')
        try:
            if math.isnan(d) or math.isinf(d) or d < 0:
                return float('inf')
        except Exception:
            return float('inf')
        return float(d)

    def _parse_tl_tokens(self):
        tl = self.tl_state_confirmed or 'None'
        tokens = [t.strip().lower() for t in tl.split(',') if t and isinstance(t, str)]
        return set(tokens)

    def _is_go_signal(self) -> bool:
        tokens = self._parse_tl_tokens()
        if 'green' in tokens:
            return True
        return False

    def _must_stop(self) -> bool:
        tokens = self._parse_tl_tokens()
        if 'red' in tokens:
            return True
        if self.yellow_treat_as_red and 'yellow' in tokens:
            return True
        return False

    def _is_stopline_relevant(self) -> bool:
        t = (self.stopline_type or '').strip().lower()
        return t in ('traffic_straight', 'traffic_left', 'traffic_right')

    def _update_state_machine(self, d: float):
        now = self.get_clock().now()

        # If stopline is not relevant (no_stopline/nonstop), always pass through
        if not self._is_stopline_relevant():
            self.fsm_state = STATE_APPROACH
            return

        if self.fsm_state == STATE_APPROACH:
            if d < self.approach_distance_threshold and self._must_stop():
                self.fsm_state = STATE_DECELERATE

        elif self.fsm_state == STATE_DECELERATE:
            x_stop = d - self.stopline_margin
            if x_stop <= 0.0 or (self.v is not None and self.v < self.v_stop_threshold):
                self.fsm_state = STATE_STOPPED
                self.last_stop_time = now
            elif self._is_go_signal() and d > max(3.0, self.clear_distance):
                # If it turns green early and we're far enough, resume approach
                self.fsm_state = STATE_APPROACH

        elif self.fsm_state == STATE_STOPPED:
            # Hold at least min_hold_time_at_red
            hold_ok = False
            try:
                if self.last_stop_time is not None:
                    held = (now - self.last_stop_time).nanoseconds * 1e-9
                    hold_ok = held >= self.min_hold_time_at_red
            except Exception:
                hold_ok = True

            green_stable = False
            try:
                if self.tl_last_seen is not None:
                    stable = (now - self.tl_last_seen).nanoseconds * 1e-9
                    green_stable = self._is_go_signal() and (stable >= self.green_confirm_time)
            except Exception:
                green_stable = self._is_go_signal()

            if hold_ok and green_stable:
                self.fsm_state = STATE_GO

        elif self.fsm_state == STATE_GO:
            if d > self.clear_distance or math.isinf(d):
                self.fsm_state = STATE_APPROACH

    def _compute_target_velocity(self, d: float) -> float:
        v_cruise = self.target_vel
        if self.fsm_state == STATE_STOPPED:
            return 0.0
        if self.fsm_state == STATE_DECELERATE:
            x_stop = max(d - self.stopline_margin, 0.0)
            v_limit = math.sqrt(max(0.0, 2.0 * self.max_decel_comfort * x_stop))
            return min(v_cruise, v_limit)
        return v_cruise

    def mpc_control(self):
        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            self.get_logger().warn("차량 상태가 초기화되지 않았습니다.")
            return

        # Select spline by mode
        if self.mode == 0 or self.mode == 1:
            current_cubic_spline = self.cubic_spline_global
            if self.xs_global == [] or self.ys_global == []:
                self.get_logger().warn("Global waypoints 데이터가 없습니다.")
                return
            if current_cubic_spline is None:
                self.get_logger().warn("Global cubic spline이 초기화되지 않았습니다.")
                return
        else:
            current_cubic_spline = self.cubic_spline_local
            if self.xs_local == [] or self.ys_local == []:
                self.get_logger().warn("Local waypoints 데이터가 없습니다.")
                return
            if current_cubic_spline is None:
                self.get_logger().warn("Local cubic spline이 초기화되지 않았습니다.")
                return

        # Traffic-aware logic
        d = self._get_effective_stopline_distance()
        self._update_state_machine(d)

        # Adjust target velocity prior to reference generation
        desired_target_vel = self._compute_target_velocity(d)
        self.target_vel = float(desired_target_vel)

        # Reference trajectory and solve MPC
        self.calc_current_s(current_cubic_spline)
        xref, tan_vec = self.calc_ref_trajectory(current_cubic_spline)
        x0 = np.array([self.x, self.y, self.yaw, self.v, self.s])
        obs = np.array([self.obs1_x, self.obs1_y, self.obs2_x, self.obs2_y])

        u_opt = np.zeros((N, NU))
        x_opt = np.zeros((N, NX))

        self.solver.set(0, "x", x0)
        self.solver.constraints_set(0, "lbx", x0)
        self.solver.constraints_set(0, "ubx", x0)
        self.get_logger().info(f"Current state: {x0}")

        for i in range(N):
            self.solver.set(i, "p", np.hstack([xref[:5, i], u_opt[i, 0], tan_vec[:, i], obs]))
        self.solver.set(N, "p", np.hstack([xref[:5, -1], u_opt[-1, 0], tan_vec[:, -1], obs]))

        status = self.solver.solve()
        if status != 0:
            self.fail_count += 1
            self.get_logger().error(f"MPC Solver failed with status {status}.")
            self.prev_velocity *= 0.98
            self.set_vehicle_command(self.prev_steering_angle, self.prev_velocity)
            return

        u_opt = np.array([self.solver.get(i, "u") for i in range(N)])
        x_opt = np.array([self.solver.get(i, "x") for i in range(N)])
        self.visualize_predicted_trajectory(x_opt)

        self.steering_angle = u_opt[1, 0] - 0.14137166941
        self.velocity = x_opt[1, 3]

        # Enforce final speed according to traffic FSM as a final guard
        final_speed_limit = self._compute_target_velocity(d)
        self.velocity = float(min(self.velocity, final_speed_limit))

        self.prev_steering_angle = self.steering_angle
        self.prev_velocity = self.velocity

        self.set_vehicle_command(self.steering_angle, self.velocity)
        self.get_logger().info(f"cmd_steer: {self.steering_angle * 180.0 / np.pi:.2f} deg, cmd_vel: {self.velocity:.2f} m/s")

    def set_vehicle_command(self, steering_angle, velocity):
        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle

        # Additional safety: if definitely stopped state, force zero speed
        if self.fsm_state == STATE_STOPPED:
            cmd.drive.speed = 0.0

        self.erp_pub.publish(cmd)
        self.publish_overlay_text()
        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")

    # ------------------------------ Visualization ------------------------------
    def publish_overlay_text(self):
        text_msg = OverlayText()
        text_msg.width = 500
        text_msg.height = 240
        text_msg.text_size = 15.0
        text_msg.line_width = 2

        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)
        text_msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        state_str = {STATE_APPROACH: 'APPROACH', STATE_DECELERATE: 'DECELERATE', STATE_STOPPED: 'STOPPED', STATE_GO: 'GO'}.get(self.fsm_state, 'UNKNOWN')
        d = self._get_effective_stopline_distance()
        tl = self.tl_state_confirmed
        stype = self.stopline_type or 'no_stopline'

        text_msg.text = f"Velocity: {self.velocity:.2f}m/s \n Steer: {self.steering_angle * 180.0 / np.pi:.2f}deg\n" \
                        f"Fail Count: {self.fail_count}\nPrev input: {self.prev_steering_angle * 180.0 / np.pi:.2f} deg, {self.prev_velocity:.2f} m/s\n" \
                        f"Mode: {self.mode} ({self.mode_description})\nState: {state_str}\nTL: {tl}\nStopline: {stype}\nStopline_d: {d:.2f} m"

        self.overlay_pub.publish(text_msg)

    def visualize_predicted_trajectory(self, x_opt):
        marker_array = MarkerArray()
        for i in range(x_opt.shape[0]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "predicted_points"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x_opt[i, 0]
            marker.pose.position.y = x_opt[i, 1]
            marker.pose.position.z = 0.0
            yaw = x_opt[i, 2]
            quaternion = yaw_to_quaternion(yaw)
            marker.pose.orientation.x = quaternion.x
            marker.pose.orientation.y = quaternion.y
            marker.pose.orientation.z = quaternion.z
            marker.pose.orientation.w = quaternion.w
            marker.scale.x = 0.3
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.mpc_predict_pub.publish(marker_array)

    def visualize_ref_trajectory(self, xref):
        marker_array = MarkerArray()
        for i in range(xref.shape[1]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "xref_points"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = xref[0, i]
            marker.pose.position.y = xref[1, i]
            marker.pose.position.z = 0.0
            yaw = xref[2, i]
            quaternion = yaw_to_quaternion(yaw)
            marker.pose.orientation.x = quaternion.x
            marker.pose.orientation.y = quaternion.y
            marker.pose.orientation.z = quaternion.z
            marker.pose.orientation.w = quaternion.w
            marker.scale.x = 0.3
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.mpc_ref_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        control = Control()
        rclpy.spin(control)
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


