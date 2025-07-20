#!/bin/bash

echo "🚗 Mode Selector 테스트 시작"
echo "================================"

# 현재 디렉토리를 workspace로 변경
cd /home/kmkm/autocar-erp

# 패키지 빌드
echo "📦 패키지 빌드 중..."
colcon build --packages-select path_planning planning_msgs

# 소스 설정
echo "🔧 환경 설정 중..."
source install/setup.bash

# 테스트 실행
echo "🧪 Mode Selector 테스트 실행 중..."
echo "Ctrl+C로 종료할 수 있습니다."
echo ""

# Launch 파일 실행
ros2 launch path_planning mode_selector_test_launch.py 