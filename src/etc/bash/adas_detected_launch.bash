#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/opt/ros/foxy/bin/ros2 launch demo_pp_yolov5 pp_yolov5_infer_launch.py 
