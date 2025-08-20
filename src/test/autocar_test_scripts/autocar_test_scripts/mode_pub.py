#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from planning_msgs.msg import ModeState
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont
import threading

class ModePublisher(Node):
    def __init__(self):
        super().__init__('mode_publisher')
        
        # Publisher 생성
        self.mode_pub = self.create_publisher(ModeState, '/mode_state', 10)
        
        # 현재 모드 상태
        self.current_mode = 0
        self.mode_descriptions = {
            0: "DRIVE",
            1: "PAUSE", 
            2: "OBSTACLE_STATIC",
            3: "OBSTACLE_DYNAMIC",
            4: "DELIVERY",
            5: "PARKING",
            6: "RETURN"
        }
        
        # 주기적으로 모드 상태 퍼블리시 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_mode_state)
        
        self.get_logger().info('Mode Publisher Node가 시작되었습니다.')
    
    def set_mode(self, mode):
        """모드 설정"""
        if mode in self.mode_descriptions:
            self.current_mode = mode
            self.get_logger().info(f'모드가 {self.mode_descriptions[mode]}({mode})로 변경되었습니다.')
    
    def publish_mode_state(self):
        """현재 모드 상태 퍼블리시"""
        msg = ModeState()
        msg.current_mode = self.current_mode
        msg.description = self.mode_descriptions[self.current_mode]
        
        self.mode_pub.publish(msg)

class ModeGUI(QWidget):
    # Qt Signal 정의
    mode_changed = pyqtSignal(int)
    
    def __init__(self, mode_publisher):
        super().__init__()
        self.mode_publisher = mode_publisher
        self.mode_descriptions = {
            0: "DRIVE",
            1: "PAUSE", 
            2: "OBSTACLE_STATIC",
            3: "OBSTACLE_DYNAMIC", 
            4: "DELIVERY",
            5: "PARKING",
            6: "RETURN"
        }
        self.current_mode = 0
        
        self.init_ui()
        
        # Signal 연결
        self.mode_changed.connect(self.on_mode_changed)
    
    def init_ui(self):
        """GUI 초기화"""
        self.setWindowTitle('Mode Publisher Control')
        self.setGeometry(100, 100, 400, 600)
        
        # 메인 레이아웃
        main_layout = QVBoxLayout()
        
        # 제목 라벨
        title_label = QLabel('Mode Publisher Control')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("QLabel { color: #2c3e50; margin: 10px; }")
        main_layout.addWidget(title_label)
        
        # 현재 모드 표시
        self.current_mode_label = QLabel(f'Current Mode: {self.mode_descriptions[self.current_mode]}')
        current_mode_font = QFont()
        current_mode_font.setPointSize(12)
        current_mode_font.setBold(True)
        self.current_mode_label.setFont(current_mode_font)
        self.current_mode_label.setStyleSheet("""
            QLabel { 
                color: #27ae60; 
                background-color: #ecf0f1;
                padding: 10px;
                border-radius: 5px;
                margin: 5px;
            }
        """)
        main_layout.addWidget(self.current_mode_label)
        
        # 구분선
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line)
        
        # 모드 버튼들
        button_layout = QVBoxLayout()
        
        self.mode_buttons = {}
        button_colors = {
            0: "#3498db",  # DRIVE - 파란색
            1: "#e74c3c",  # PAUSE - 빨간색  
            2: "#f39c12", # OBSTACLE_STATIC - 주황색
            3: "#e67e22", # OBSTACLE_DYNAMIC - 진한 주황색
            4: "#9b59b6", # DELIVERY - 보라색
            5: "#1abc9c", # PARKING - 청록색
            6: "#34495e"  # RETURN - 회색
        }
        
        for mode, description in self.mode_descriptions.items():
            button = QPushButton(f'{description} (Mode {mode})')
            button.setMinimumHeight(50)
            button.setFont(QFont("Arial", 10, QFont.Bold))
            
            # 버튼 스타일 설정
            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: {button_colors[mode]};
                    color: white;
                    border: none;
                    border-radius: 8px;
                    padding: 10px;
                    margin: 3px;
                }}
                QPushButton:hover {{
                    background-color: {self.darken_color(button_colors[mode])};
                }}
                QPushButton:pressed {{
                    background-color: {self.darken_color(button_colors[mode], 0.3)};
                }}
            """)
            
            # 버튼 클릭 이벤트 연결
            button.clicked.connect(lambda checked, m=mode: self.on_mode_button_clicked(m))
            
            self.mode_buttons[mode] = button
            button_layout.addWidget(button)
        
        main_layout.addLayout(button_layout)
        
        # 스트레치 추가 (하단 여백)
        main_layout.addStretch()
        
        # 정보 라벨
        info_label = QLabel('버튼을 클릭하여 모드를 변경하세요.')
        info_label.setStyleSheet("QLabel { color: #7f8c8d; font-style: italic; margin: 10px; }")
        main_layout.addWidget(info_label)
        
        self.setLayout(main_layout)
        
        # 초기 모드 버튼 활성화 표시
        self.update_button_states()
    
    def darken_color(self, hex_color, factor=0.2):
        """색상을 어둡게 만드는 헬퍼 함수"""
        hex_color = hex_color.lstrip('#')
        rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        darkened = tuple(int(c * (1 - factor)) for c in rgb)
        return f"#{darkened[0]:02x}{darkened[1]:02x}{darkened[2]:02x}"
    
    def on_mode_button_clicked(self, mode):
        """모드 버튼 클릭 핸들러"""
        self.mode_changed.emit(mode)
    
    def on_mode_changed(self, mode):
        """모드 변경 시 호출되는 슬롯"""
        self.current_mode = mode
        self.mode_publisher.set_mode(mode)
        
        # 현재 모드 라벨 업데이트
        self.current_mode_label.setText(f'Current Mode: {self.mode_descriptions[mode]} ({mode})')
        
        # 버튼 상태 업데이트
        self.update_button_states()
    
    def update_button_states(self):
        """현재 활성 모드에 따라 버튼 상태 업데이트"""
        for mode, button in self.mode_buttons.items():
            if mode == self.current_mode:
                # 현재 모드 버튼은 더 진하게 표시
                button.setStyleSheet(button.styleSheet() + """
                    QPushButton {
                        border: 3px solid #2c3e50;
                        font-weight: bold;
                    }
                """)
            else:
                # 기본 스타일로 복원
                button_colors = {
                    0: "#3498db", 1: "#e74c3c", 2: "#f39c12", 3: "#e67e22", 
                    4: "#9b59b6", 5: "#1abc9c", 6: "#34495e"
                }
                button.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {button_colors[mode]};
                        color: white;
                        border: none;
                        border-radius: 8px;
                        padding: 10px;
                        margin: 3px;
                    }}
                    QPushButton:hover {{
                        background-color: {self.darken_color(button_colors[mode])};
                    }}
                    QPushButton:pressed {{
                        background-color: {self.darken_color(button_colors[mode], 0.3)};
                    }}
                """)

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # QApplication 생성
    app = QApplication(sys.argv)
    
    # ROS2 노드 생성
    mode_publisher = ModePublisher()
    
    # GUI 생성
    gui = ModeGUI(mode_publisher)
    gui.show()
    
    # ROS2 스핀을 위한 별도 스레드
    def ros_spin():
        rclpy.spin(mode_publisher)
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    try:
        # Qt 이벤트 루프 실행
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        # 정리
        mode_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
