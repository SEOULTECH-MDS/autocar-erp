#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from mpc_acados import create_ocp, set_reference_trajectory  
from acados_template import AcadosOcpSolver

class MPCPathTracker(Node):
    def __init__(self):
        super().__init__('mpc_path_tracker')

        # 퍼블리셔 생성
        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        # 서브스크라이버 생성
        self.localisation_sub = self.create_subscription(PoseStamped, '/autocar/location', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path, '/autocar/path', self.path_cb, 10)

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.vel = 0.0
        self.ref_path = None  # 참조 경로 저장

        # MPC 설정
        self.ocp = create_ocp()
        self.ocp_solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        # 타이머 설정
        self.timer = self.create_timer(0.2, self.timer_cb)  # 5Hz 주기로 실행

    # 타이머 콜백 함수
    def timer_cb(self):
        self.mpc_control()

    # 차량 상태 콜백 함수
    def vehicle_state_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.vel = msg.twist.linear.x if hasattr(msg, 'twist') else 0.0

    # 경로 데이터 수신 콜백 함수
    def path_cb(self, msg):
        # Path 메시지를 참조 경로로 변환
        ref_path = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            ref_path.append([x, y, yaw])
        self.ref_path = np.array(ref_path)

    # MPC 제어 알고리즘 실행
    def mpc_control(self):
        if self.x is None or self.y is None or self.ref_path is None or len(self.ref_path) == 0:
            return

        # 현재 상태 설정
        x0 = np.array([self.x, self.y, self.yaw, self.vel])
        self.ocp_solver.set(0, "lbx", x0)
        self.ocp_solver.set(0, "ubx", x0)

        # 참조 궤적 설정
        try:
            set_reference_trajectory(self.ocp_solver, self.ref_path[:self.ocp.dims.N + 1], target_speed=1.0)
        except IndexError:
            self.get_logger().error("참조 경로가 예측 단계 수보다 짧습니다.")
            return

        # MPC 문제 해결
        status = self.ocp_solver.solve()
        if status != 0:
            self.get_logger().error(f"ACADOS solver failed with status {status}")
            return

        # 최적화 결과 가져오기
        u0 = self.ocp_solver.get(0, "u")
        velocity = u0[1]
        steering_angle = u0[0]

        # 차량 명령 퍼블리시
        self.set_vehicle_command(velocity, steering_angle)

    # 차량 명령 퍼블리시 함수
    def set_vehicle_command(self, velocity, steering_angle):
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.angular.z = steering_angle
        self.tracker_pub.publish(cmd_vel)

        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle
        self.erp_pub.publish(cmd)

        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")

def main(args=None):
    rclpy.init(args=args)
    try:
        mpc_tracker = MPCPathTracker()
        rclpy.spin(mpc_tracker)
    finally:
        mpc_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()