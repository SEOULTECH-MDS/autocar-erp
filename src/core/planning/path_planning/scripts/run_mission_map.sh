#!/bin/bash

echo "🗺️ 맵 기반 미션 시스템 시작..."

# 기존 프로세스 종료
pkill -f tf_broadcaster
pkill -f mode_selector_simple
pkill -f simple_test
pkill -f mode_selector_visualizer
pkill -f mission_map
pkill -f rviz2

sleep 2

# 환경 설정
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# 프로젝트 루트 디렉토리 찾기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../../../../" && pwd)"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

# 1. TF 브로드캐스터 실행
echo "🗺️ TF 브로드캐스터 시작..."
"$PROJECT_ROOT/install/path_planning/bin/tf_broadcaster" &
TF_PID=$!

sleep 1

# 2. 모드 셀렉터 실행
echo "🧠 모드 셀렉터 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mode_selector_simple" &
MODE_PID=$!

sleep 1

# 3. 맵 기반 미션 시스템 실행
echo "🗺️ 맵 기반 미션 시스템 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mission_map" &
MISSION_PID=$!

sleep 2

# 4. 시각화 노드 실행
echo "🎨 시각화 노드 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mode_selector_visualizer" &
VISUAL_PID=$!

sleep 3

# 5. RViz 실행
echo "🖥️ RViz 시작..."
rviz2 -d "$PROJECT_ROOT/src/core/planning/path_planning/src/test_utils/mode_selector.rviz" &
RVIZ_PID=$!

echo "✅ 모든 노드가 실행되었습니다!"
echo "🗺️ 맵에서 차량이 주행하며 미션을 수행합니다."
echo "📍 미션 포인트들이 3D 마커로 표시됩니다."
echo "🛣️ 차량 경로가 노란색 선으로 표시됩니다."
echo ""
echo "종료하려면 Ctrl+C를 누르세요."

# 프로세스 종료 함수
cleanup() {
    echo ""
    echo "🛑 시스템 종료 중..."
    kill $TF_PID $MODE_PID $MISSION_PID $VISUAL_PID $RVIZ_PID 2>/dev/null
    pkill -f tf_broadcaster
    pkill -f mode_selector_simple
    pkill -f mission_map
    pkill -f mode_selector_visualizer
    pkill -f rviz2
    echo "✅ 모든 프로세스가 종료되었습니다."
    exit 0
}

# Ctrl+C 시그널 처리
trap cleanup SIGINT

# 프로세스 모니터링
while true; do
    sleep 1
    if ! kill -0 $TF_PID 2>/dev/null; then
        echo "❌ TF 브로드캐스터가 종료되었습니다."
        cleanup
    fi
    if ! kill -0 $MODE_PID 2>/dev/null; then
        echo "❌ 모드 셀렉터가 종료되었습니다."
        cleanup
    fi
    if ! kill -0 $MISSION_PID 2>/dev/null; then
        echo "❌ 미션 맵이 종료되었습니다."
        cleanup
    fi
done 