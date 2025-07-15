
import os

import ament_index_python.packages
import launch
import launch_ros.actions
import yaml

def generate_launch_description():
  share_dir = ament_index_python.packages.get_package_share_directory('nmea_ros_bridge')
  params_file = os.path.join(share_dir, 'config', 'udp_config.yaml')
  with open(params_file, 'r') as f:
    params = yaml.safe_load(f)['nmea_udp']['ros__parameters']
  nmea_udp = launch_ros.actions.Node(package='nmea_ros_bridge',
                                      executable='nmea_udp',
                                      namespace='navsat',
                                      output='screen',
                                      parameters=[params])

  return launch.LaunchDescription([nmea_udp,
                                  launch.actions.RegisterEventHandler(
                                      event_handler=launch.event_handlers.OnProcessExit(
                                          target_action=nmea_udp,
                                          on_exit=[launch.actions.EmitEvent(
                                              event=launch.events.Shutdown())],
                                      )),
                                  ])