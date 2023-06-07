#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/home/nvidia/mdl_ws/install/lslidar_driver/lib/lslidar_driver/lslidar_c16driver_node --ros-args -r __node:=lslidar_driver_node -r __ns:=/c16 --params-file /home/nvidia/mdl_ws/install/lslidar_driver/share/lslidar_driver/params/lslidar_c16.yaml

