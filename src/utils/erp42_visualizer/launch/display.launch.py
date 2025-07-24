import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Launch 구성 값 가져오기
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 패키지 경로 찾기
    pkg_path = get_package_share_directory('erp42_visualizer')
    autocar_tf_path = get_package_share_directory('localization_core')

    # URDF(XACRO) 파일 경로 설정
    xacro_file = os.path.join(pkg_path, 'urdf', 'erp42.urdf.xacro')

    # XACRO를 사용하여 URDF 로봇 설명을 생성하고 ParameterValue로 감싸기
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = doc.toxml()
    
    robot_description_content = ParameterValue(robot_description_xml, value_type=str)
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    
    autocar_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autocar_tf_path, 'launch', 'autocar_tf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # robot_state_publisher 노드 설정
    # URDF를 읽고 /robot_description 토픽으로 게시하며, TF 변환을 발행합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Node to convert Ackermann commands to joint states
    ackermann_to_joint_state_node = Node(
        package='erp42_visualizer',
        executable='ackermann_to_joint_states.py',
        name='ackermann_to_joint_state_publisher'
    )

    
    # 실행할 노드들을 포함하는 LaunchDescription 생성
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        autocar_tf_launch,

        TimerAction(
            period=1.0,
            actions=[
                robot_state_publisher_node,
                ackermann_to_joint_state_node
            ]
        )
    ])