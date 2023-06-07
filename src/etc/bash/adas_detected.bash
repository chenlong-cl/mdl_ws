#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/home/nvidia/mdl_ws/install/demo_pp_yolov5/lib/demo_pp_yolov5/demo_pp_yolov5 --ros-args -r __node:=demo_pp_yolov5 --params-file /home/nvidia/mdl_ws/src/etc/yaml/mdl_adas.yaml


