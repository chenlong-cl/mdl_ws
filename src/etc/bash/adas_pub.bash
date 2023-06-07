#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/home/nvidia/mdl_ws/install/camera_ros/lib/camera_ros/camera_ros_node --ros-args -r __node:=camera_ros_node -r __ns:=/adas --params-file /home/nvidia/mdl_ws/src/etc/yaml/camera_ros_adas.yaml

