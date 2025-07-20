# nmea_ros_bridge

## Overview
nmea_ros_bridge converts the NMEA of a GNSS receiver to a ROS topic.

Requires [nmea_msgs](http://wiki.ros.org/nmea_msgs).

## Install

1) First, download the ros2ized nmea_msgs and nmea_ros_bridge to your colcon_ws directory.

        cd $HOME/colcon_ws/src
        sudo apt-get install ros-foxy-nmea-msgs
        git clone -b ros2-v0.1.0 https://github.com/MapIV/nmea_ros_bridge.git

2) Build nmea_ros_bridge

        cd $HOME/colcon_ws/
        colcon build

## Usage
1) Connect the GNSS receiver and start nmea_ros_bridge(tcp).

        cd $HOME/colcon_ws/
        source install/setup.bash
        ros2 launch nmea_ros_bridge nmea_tcp.launch.py

2) Connect the GNSS receiver and start nmea_ros_bridge(udp).

        cd $HOME/colcon_ws/
        source install/setup.bash
        ros2 launch nmea_ros_bridge nmea_udp.launch.py
