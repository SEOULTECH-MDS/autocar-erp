#!/bin/bash

echo "🗺️ 맵 기반 미션 시스템 시작..."

# 기존 프로세스 종료
echo "🧹 기존 프로세스 정리 중..."
pkill -f tf_broadcaster 2>/dev/null
pkill -f mode_selector_simple 2>/dev/null
pkill -f simple_test 2>/dev/null
pkill -f mode_selector_visualizer 2>/dev/null
pkill -f mission_map 2>/dev/null
pkill -f rviz2 2>/dev/null

sleep 2

# 환경 설정
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# 프로젝트 루트 디렉토리 찾기
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../../../../" && pwd)"

# ROS2 환경 설정
echo "🔧 ROS2 환경 설정 중..."
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

# 실행 파일 존재 확인
echo "🔍 실행 파일 확인 중..."
if [ ! -f "$PROJECT_ROOT/install/path_planning/bin/tf_broadcaster" ]; then
    echo "❌ tf_broadcaster 실행 파일을 찾을 수 없습니다."
    exit 1
fi

if [ ! -f "$PROJECT_ROOT/install/path_planning/bin/mode_selector_simple" ]; then
    echo "❌ mode_selector_simple 실행 파일을 찾을 수 없습니다."
    exit 1
fi

if [ ! -f "$PROJECT_ROOT/install/path_planning/bin/mission_map" ]; then
    echo "❌ mission_map 실행 파일을 찾을 수 없습니다."
    exit 1
fi

if [ ! -f "$PROJECT_ROOT/install/path_planning/bin/mode_selector_visualizer" ]; then
    echo "❌ mode_selector_visualizer 실행 파일을 찾을 수 없습니다."
    exit 1
fi

echo "✅ 모든 실행 파일이 확인되었습니다."

# 1. TF 브로드캐스터 실행
echo "🗺️ TF 브로드캐스터 시작..."
"$PROJECT_ROOT/install/path_planning/bin/tf_broadcaster" &
TF_PID=$!

# TF 브로드캐스터 시작 확인
sleep 2
if ! kill -0 $TF_PID 2>/dev/null; then
    echo "❌ TF 브로드캐스터 시작 실패"
    exit 1
fi
echo "✅ TF 브로드캐스터 시작됨 (PID: $TF_PID)"

# 2. 모드 셀렉터 실행
echo "🧠 모드 셀렉터 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mode_selector_simple" &
MODE_PID=$!

# 모드 셀렉터 시작 확인
sleep 2
if ! kill -0 $MODE_PID 2>/dev/null; then
    echo "❌ 모드 셀렉터 시작 실패"
    kill $TF_PID 2>/dev/null
    exit 1
fi
echo "✅ 모드 셀렉터 시작됨 (PID: $MODE_PID)"

# 3. 맵 기반 미션 시스템 실행
echo "🗺️ 맵 기반 미션 시스템 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mission_map" &
MISSION_PID=$!

# 미션 맵 시작 확인
sleep 3
if ! kill -0 $MISSION_PID 2>/dev/null; then
    echo "❌ 미션 맵 시작 실패"
    kill $TF_PID $MODE_PID 2>/dev/null
    exit 1
fi
echo "✅ 미션 맵 시작됨 (PID: $MISSION_PID)"

# 4. 시각화 노드 실행
echo "🎨 시각화 노드 시작..."
"$PROJECT_ROOT/install/path_planning/bin/mode_selector_visualizer" &
VISUAL_PID=$!

# 시각화 노드 시작 확인
sleep 2
if ! kill -0 $VISUAL_PID 2>/dev/null; then
    echo "❌ 시각화 노드 시작 실패"
    kill $TF_PID $MODE_PID $MISSION_PID 2>/dev/null
    exit 1
fi
echo "✅ 시각화 노드 시작됨 (PID: $VISUAL_PID)"

# 5. RViz 실행
echo "🖥️ RViz 시작..."
rviz2 -d "$PROJECT_ROOT/src/core/planning/path_planning/src/test_utils/mode_selector.rviz" &
RVIZ_PID=$!

# RViz 시작 확인
sleep 3
if ! kill -0 $RVIZ_PID 2>/dev/null; then
    echo "❌ RViz 시작 실패"
    kill $TF_PID $MODE_PID $MISSION_PID $VISUAL_PID 2>/dev/null
    exit 1
fi
echo "✅ RViz 시작됨 (PID: $RVIZ_PID)"

echo ""
echo "🎉 모든 노드가 성공적으로 실행되었습니다!"
echo "🗺️ 맵에서 차량이 주행하며 미션을 수행합니다."
echo "📍 미션 포인트들이 3D 마커로 표시됩니다."
echo "🛣️ 차량 경로가 노란색 선으로 표시됩니다."
echo ""
echo "📊 실행 중인 프로세스:"
echo "   TF 브로드캐스터: $TF_PID"
echo "   모드 셀렉터: $MODE_PID"
echo "   미션 맵: $MISSION_PID"
echo "   시각화 노드: $VISUAL_PID"
echo "   RViz: $RVIZ_PID"
echo ""
echo "종료하려면 Ctrl+C를 누르세요."

# 프로세스 종료 함수
cleanup() {
    echo ""
    echo "🛑 시스템 종료 중..."
    
    # 개별 프로세스 종료
    kill $TF_PID 2>/dev/null
    kill $MODE_PID 2>/dev/null
    kill $MISSION_PID 2>/dev/null
    kill $VISUAL_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
    
    # 추가 정리
    pkill -f tf_broadcaster 2>/dev/null
    pkill -f mode_selector_simple 2>/dev/null
    pkill -f mission_map 2>/dev/null
    pkill -f mode_selector_visualizer 2>/dev/null
    pkill -f rviz2 2>/dev/null
    
    echo "✅ 모든 프로세스가 종료되었습니다."
    exit 0
}

# Ctrl+C 시그널 처리
trap cleanup SIGINT

# 간단한 상태 모니터링 (10초마다)
echo "📈 시스템 모니터링 시작..."
while true; do
    sleep 10
    
    # 프로세스 상태 확인
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
    if ! kill -0 $VISUAL_PID 2>/dev/null; then
        echo "❌ 시각화 노드가 종료되었습니다."
        cleanup
    fi
    
    echo "✅ 모든 노드가 정상 실행 중..."
done 