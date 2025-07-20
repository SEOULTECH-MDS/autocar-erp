#!/bin/bash

# 프로젝트 루트 디렉토리 찾기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../../../../" && pwd)"

# ROS 환경 설정
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

echo "🚀 RViz 시각화 시작..."

# 기존 노드들 종료
pkill -f tf_broadcaster
pkill -f mode_selector_simple
pkill -f simple_test
pkill -f mode_selector_visualizer
pkill -f rviz2

sleep 2

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

# 3. 테스트 노드 실행
echo "🧪 테스트 노드 시작..."
"$PROJECT_ROOT/install/path_planning/bin/simple_test" &
TEST_PID=$!

sleep 1

# 4. 시각화 노드 실행
echo "🎨 시각화 노드 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mode_selector_visualizer" &
VISUAL_PID=$!

sleep 3

# 5. RViz 실행 (더 오래 기다린 후 실행)
echo "🖥️ RViz 시작..."
rviz2 -d "$PROJECT_ROOT/src/core/planning/path_planning/src/test_utils/mode_selector.rviz" &
RVIZ_PID=$!

echo "✅ 모든 노드가 실행되었습니다!"
echo "RViz 창에서 3D 시각화를 확인하세요."
echo ""
echo "종료하려면 Ctrl+C를 누르세요."

# 대기
wait

# 종료 시 모든 프로세스 정리
echo "🛑 모든 노드를 종료합니다..."
kill $TF_PID $MODE_PID $TEST_PID $VISUAL_PID $RVIZ_PID 2>/dev/null
pkill -f tf_broadcaster
pkill -f mode_selector_simple
pkill -f simple_test
pkill -f mode_selector_visualizer
pkill -f rviz2

echo "✅ 종료 완료!" 