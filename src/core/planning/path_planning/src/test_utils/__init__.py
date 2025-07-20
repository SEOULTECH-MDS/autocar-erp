# Test utilities for mode selector
"""
Test Utilities Package - 모드 셀렉터 테스트 도구 모음

이 패키지는 모드 셀렉터의 테스트 및 시각화를 위한 다양한 도구들을 포함합니다.

주요 구성 요소:

1. 테스트 노드들:
   - simple_test_node.py: 기본 센서 데이터 시뮬레이션
   - mode_selector_test_node.py: 고급 테스트 기능
   - dynamic_vehicle_test.py: 동적 차량 시뮬레이션

2. 시각화 노드들:
   - mode_selector_visualizer.py: 기본 RViz 시각화
   - enhanced_visualizer.py: 고급 시각화 기능

3. 지원 노드들:
   - tf_broadcaster.py: 좌표계 변환 브로드캐스터

4. 실행 파일들:
   - complete_visualization_launch.py: 통합 실행
   - simple_test_launch.py: 간단한 테스트 실행
   - visualization_launch.py: 시각화 전용 실행
   - mode_selector_test_launch.py: 고급 테스트 실행

5. 설정 파일들:
   - mode_selector.rviz: RViz 설정 파일

사용법:
    # 전체 테스트 환경 실행
    ./run_visualization.sh
    
    # 또는 개별 실행
    ros2 launch path_planning complete_visualization_launch.py

작성자: BAE Team
버전: 1.0
""" 