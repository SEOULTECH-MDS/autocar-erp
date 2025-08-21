from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package share directory
    erp42_visualizer_share_dir = get_package_share_directory('erp42_visualizer')

    # --- Launch Arguments ---
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        # default_value='kcity_v5',
        default_value='mirae_map',
        description='Name of the map folder in localization_core/data'
    )
    map_osm_file_arg = DeclareLaunchArgument(
        'map_osm_file',
        default_value='lanelet2_map.osm',
        description='Name of the .osm file in the map folder'
    )
    map_origin_lat_arg = DeclareLaunchArgument(
        'map_origin_lat',
        # default_value='37.239205', # kcity 
        default_value='37.6301124677', # mirae 
        description='Latitude of map origin for UTM projection'
    )
    map_origin_lon_arg = DeclareLaunchArgument(
        'map_origin_lon',
        # default_value='126.773193', # kcity
        default_value='127.08146372752', # mirae
        description='Longitude of map origin for UTM projection'
    )

    # Define the path to the map file using substitutions
    map_path_substitution = PathJoinSubstitution([
        FindPackageShare('localization_core'), 'data', LaunchConfiguration('map_name'),
        LaunchConfiguration('map_osm_file')
    ])

    rviz_config_path = PathJoinSubstitution([FindPackageShare('autocar_utils'), 'rviz', 'map.rviz'])

    # Define paths for sample_map test
    xacro_file = os.path.join(erp42_visualizer_share_dir, 'urdf', 'erp42.urdf.xacro')

    # XACRO processing
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = doc.toxml()
    robot_description_content = ParameterValue(robot_description_xml, value_type=str)
    
    robot_description_params = {'robot_description': robot_description_content, 'use_sim_time': False}

    return LaunchDescription([
        # --- Arguments ---
        map_name_arg,
        map_osm_file_arg,
        map_origin_lat_arg,
        map_origin_lon_arg,
        DeclareLaunchArgument(
            'lanelet2_map_path',
            default_value=map_path_substitution,
            description='Path to the lanelet2 map file'
        ),

        # --- Nodes ---
        # 1. Vehicle TF Publisher (world -> map -> base_link)
        Node(
            package='localization_core',
            executable='autocar_tf_publisher',
            name='autocar_tf_publisher',
            parameters=[{
                'map_origin_lat': LaunchConfiguration('map_origin_lat'),
                'map_origin_lon': LaunchConfiguration('map_origin_lon')
            }]
        ),

        # 2. Map Loader Node
        Node(
            package='autoware_map_loader',
            executable='autoware_lanelet2_map_loader',
            name='lanelet2_map_loader',
            parameters=[{
                'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                "map_projector_type": "UTM",
                "latitude": LaunchConfiguration('map_origin_lat'),
                "longitude": LaunchConfiguration('map_origin_lon')
            }]
        ),

        # 3. Map Visualizer Node (Publishing markers in 'world' frame)
        Node(
            package='autoware_lanelet2_map_visualizer',
            executable='lanelet2_map_visualization',
            name='lanelet2_map_visualizer',
            parameters=[{'map_frame': 'world'}],
            remappings=[
                ('input/lanelet2_map', '/map/vector_map'),
                ('output/lanelet2_map_marker', '/map/lanelet2_map_viz')
            ]
        ),

        # 4. Robot State Publisher for vehicle model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_params]
        ),
        
        # 5. Ackermann to Joint States for vehicle model
        Node(
            package='erp42_visualizer',
            executable='ackermann_to_joint_states.py',
            name='ackermann_to_joint_state_publisher'
        ),

        # 6. Lanelet Click Planner
        Node(
            package='localization_core',
            executable='global_click_planner',
            name='lanelet_click_planner',
            parameters=[{
                'map_origin.lat': LaunchConfiguration('map_origin_lat'),
                'map_origin.lon': LaunchConfiguration('map_origin_lon'),
                'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                'allowed_lanelet_subtypes': ['road']
            }],
            output='screen'
        ),

        # 6.5 Current lanelet detection (localization)
        Node(
            package='localization_core',
            executable='localization',
            name='localization',
            parameters=[{
                'map_origin.lat': LaunchConfiguration('map_origin_lat'),
                'map_origin.lon': LaunchConfiguration('map_origin_lon'),
                'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                'allowed_lanelet_subtypes': ['road'],
                'max_select_distance': 3.0
            }],
            output='screen'
        ),

        # 7. RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ]) 