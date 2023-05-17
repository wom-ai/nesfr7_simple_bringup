#!/bin/bash
source /opt/ros/galactic/setup.bash
source /home/wom/ros_ws/install/setup.bash
ros2 launch nesfr7_simple_bringup nesfr7_simple.launch.py namespace:=nesfr7_01
