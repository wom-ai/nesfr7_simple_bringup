#!/bin/bash
set -x

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterface>wlan0</></></></>'

source /opt/ros/galactic/setup.bash
source /usr/local/bros/release_profile.sh
source /home/wom/ros_ws/install/setup.bash
ros2 launch nesfr7_simple_bringup nesfr7_simple.launch.py namespace:=nesfr7_01
