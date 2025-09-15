#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import time

class MPCRealTimePlotter(Node):
    def __init__(self):
        super().__init__('mpc_realtime_plotter')
        
        # Subscribers
        self.mpc_ref_sub = self.create_subscription(
            MarkerArray, '/autocar/mpc_ref', self.mpc_ref_cb, 10
        )
        self.mpc_predict_sub = self.create_subscription(
            MarkerArray, '/autocar/mpc_predict', self.mpc_predict_cb, 10
        )
        self.erp_cmd_sub = self.create_subscription(
            AckermannDriveStamped, '/erp/cmd_vel', self.erp_cmd_cb, 10
        )
        
        # 데이터 저장용 deque (최대 1000개 포인트)
        self.max_points = 1000
        self.ref_x_data = deque(maxlen=self.max_points)
        self.ref_y_data = deque(maxlen=self.max_points)
        self.predict_x_data = deque(maxlen=self.max_points)
        self.predict_y_data = deque(maxlen=self.max_points)
        
        # 제어 입력 데이터
        self.time_data = deque(maxlen=self.max_points)
        self.steering_data = deque(maxlen=self.max_points)
        self.velocity_data = deque(maxlen=self.max_points)
        self.steering_rate_data = deque(maxlen=self.max_points)  # 조향각 변화율
        
        # 이전 조향각과 시간 (변화율 계산용)
        self.prev_steering = None
        self.prev_time = None
        
        # 현재 시간 기록용
        self.start_time = time.time()
        
        # 스레드 안전성을 위한 락
        self.data_lock = threading.Lock()
        
        # 최신 데이터 저장
        self.latest_ref_x = []
        self.latest_ref_y = []
        self.latest_predict_x = []
        self.latest_predict_y = []
        
        # 참조 속도 데이터 저장용 deque 추가
        self.target_velocity_data = deque(maxlen=self.max_points)
        
        # Matplotlib 설정
        self.setup_plots()
        
        self.get_logger().info("MPC Real-time Plotter 초기화 완료")
    
    def setup_plots(self):
        """Matplotlib 플롯 설정"""
        # Figure와 subplots 생성 (2x2 그리드)
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('MPC Real-time Monitoring', fontsize=16)
        
        # 1. 경로 추적 플롯 (xy 평면)
        self.ax1.set_title('Path Tracking (X-Y Plane)')
        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.grid(True)
        self.ax1.set_aspect('equal')
        
        # 경로 추적 라인 초기화 - 이 부분이 누락되어 있었습니다!
        self.ref_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='Reference Path')
        self.predict_line, = self.ax1.plot([], [], 'r-', linewidth=1.5, label='Predicted Path')
        self.ax1.legend()
        
        # 2. 조향각 변화율 시간 이력
        self.ax2.set_title('Steering Rate History')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Steering Rate [deg/s]')
        self.ax2.grid(True)
        
        # 3. 조향각 시간 이력
        self.ax3.set_title('Steering Angle History')
        self.ax3.set_xlabel('Time [s]')
        self.ax3.set_ylabel('Steering Angle [deg]')
        self.ax3.grid(True)
        
        # 4. 속도 시간 이력 (참조 속도 라인 추가)
        self.ax4.set_title('Velocity History')
        self.ax4.set_xlabel('Time [s]')
        self.ax4.set_ylabel('Velocity [m/s]')
        self.ax4.grid(True)
        
        # 제어 입력 라인
        self.steering_rate_line, = self.ax2.plot([], [], 'c-', linewidth=2)
        self.steering_line, = self.ax3.plot([], [], 'g-', linewidth=2)
        self.velocity_line, = self.ax4.plot([], [], 'm-', linewidth=2, label='Actual Velocity')
        # 참조 속도 라인 추가
        self.target_velocity_line, = self.ax4.plot([], [], 'r--', linewidth=1.5, label='Target Velocity')
        self.ax4.legend()
        
        # 애니메이션 설정
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plots, interval=100, blit=False
        )
    
    def mpc_ref_cb(self, msg):
        """MPC 참조 경로 콜백"""
        current_time = time.time() - self.start_time
        
        with self.data_lock:
            if len(msg.markers) > 0:
                ref_x = [marker.pose.position.x for marker in msg.markers]
                ref_y = [marker.pose.position.y for marker in msg.markers]
                
                # 최신 참조 경로 저장
                self.latest_ref_x = ref_x
                self.latest_ref_y = ref_y
                
                # 첫 번째 포인트를 히스토리에 추가 (경로 추적용)
                if ref_x and ref_y:
                    self.ref_x_data.append(ref_x[0])
                    self.ref_y_data.append(ref_y[0])
                
                # 참조 속도 추출 (첫번째 마커에서)
                # 속도 정보가 marker.scale.z 또는 다른 필드에 저장되어 있다고 가정
                # 필요에 따라 이 부분을 조정하세요
                if len(msg.markers) > 0:
                    # 첫번째 마커의 scale.z에 속도가 저장되어 있다고 가정
                    # 실제 구현에 따라 이 부분을 수정하세요
                    target_vel = msg.markers[0].color.r * 5.0  
                    self.target_velocity_data.append((current_time, target_vel))
    
    def mpc_predict_cb(self, msg):
        """MPC 예측 경로 콜백"""
        with self.data_lock:
            if len(msg.markers) > 0:
                predict_x = [marker.pose.position.x for marker in msg.markers]
                predict_y = [marker.pose.position.y for marker in msg.markers]
                
                # 최신 예측 경로 저장
                self.latest_predict_x = predict_x
                self.latest_predict_y = predict_y
                
                # 첫 번째 포인트를 히스토리에 추가 (경로 추적용)
                if predict_x and predict_y:
                    self.predict_x_data.append(predict_x[0])
                    self.predict_y_data.append(predict_y[0])
    
    def erp_cmd_cb(self, msg):
        """ERP 제어 명령 콜백"""
        current_time = time.time() - self.start_time
        steering_deg = np.rad2deg(msg.drive.steering_angle)  # 라디안에서 도로 변환
        velocity = msg.drive.speed
        
        # 조향각 변화율 계산
        steering_rate = 0.0
        if self.prev_steering is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                steering_rate = (steering_deg - self.prev_steering) / dt
        
        with self.data_lock:
            self.time_data.append(current_time)
            self.steering_data.append(steering_deg)
            self.velocity_data.append(velocity)
            self.steering_rate_data.append(steering_rate)
        
        # 이전 값 업데이트
        self.prev_steering = steering_deg
        self.prev_time = current_time
    
    def update_plots(self, frame):
        """플롯 업데이트 함수"""
        with self.data_lock:
            # 1. 경로 추적 히스토리 업데이트
            if self.ref_x_data and self.ref_y_data:
                self.ref_line.set_data(list(self.ref_x_data), list(self.ref_y_data))
            
            if self.predict_x_data and self.predict_y_data:
                self.predict_line.set_data(list(self.predict_x_data), list(self.predict_y_data))
            
            # 축 범위 자동 조정
            if self.ref_x_data or self.predict_x_data:
                all_x = list(self.ref_x_data) + list(self.predict_x_data)
                all_y = list(self.ref_y_data) + list(self.predict_y_data)
                if all_x and all_y:
                    margin = 2.0
                    self.ax1.set_xlim(min(all_x) - margin, max(all_x) + margin)
                    self.ax1.set_ylim(min(all_y) - margin, max(all_y) + margin)
            
            # 2. 조향각 변화율 히스토리 업데이트
            if self.time_data and self.steering_rate_data:
                self.steering_rate_line.set_data(list(self.time_data), list(self.steering_rate_data))
                self.ax2.set_xlim(max(0, max(self.time_data) - 30), max(self.time_data) + 1)  # 최근 30초
                if self.steering_rate_data:
                    margin = 10.0  # deg/s
                    max_rate = max(abs(min(self.steering_rate_data)), abs(max(self.steering_rate_data)))
                    if max_rate > 0:
                        self.ax2.set_ylim(-max_rate - margin, max_rate + margin)
                    else:
                        self.ax2.set_ylim(-margin, margin)
            
            # 3. 조향각 히스토리 업데이트
            if self.time_data and self.steering_data:
                self.steering_line.set_data(list(self.time_data), list(self.steering_data))
                self.ax3.set_xlim(max(0, max(self.time_data) - 30), max(self.time_data) + 1)  # 최근 30초
                if self.steering_data:
                    margin = 5.0
                    self.ax3.set_ylim(min(self.steering_data) - margin, max(self.steering_data) + margin)
            
            # 4. 속도 히스토리 업데이트
            if self.time_data and self.velocity_data:
                self.velocity_line.set_data(list(self.time_data), list(self.velocity_data))
                
                # 참조 속도 데이터가 있으면 플롯에 추가
                if self.target_velocity_data:
                    target_times, target_vels = zip(*self.target_velocity_data)
                    self.target_velocity_line.set_data(target_times, target_vels)
                
                self.ax4.set_xlim(max(0, max(self.time_data) - 30), max(self.time_data) + 1)  # 최근 30초
                
                if self.velocity_data:
                    # 실제 속도와 참조 속도를 모두 고려하여 y축 범위 설정
                    all_vels = list(self.velocity_data)
                    if self.target_velocity_data:
                        all_vels.extend([v for _, v in self.target_velocity_data])
                        
                    margin = 0.5
                    min_vel = max(0, min(all_vels) - margin)
                    max_vel = max(all_vels) + margin
                    self.ax4.set_ylim(min_vel, max_vel)
        
        return self.ref_line, self.predict_line, self.steering_rate_line, self.steering_line, self.velocity_line, self.target_velocity_line
    
    def show_plot(self):
        """플롯 표시"""
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        plotter = MPCRealTimePlotter()
        
        # ROS 스핀을 별도 스레드에서 실행
        def spin_ros():
            rclpy.spin(plotter)
        
        import threading
        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        # 메인 스레드에서 matplotlib 실행
        plotter.show_plot()
        
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
