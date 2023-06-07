#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/home/nvidia/mdl_ws/install/dsm_detect/lib/dsm_detect/dsm_detect --ros-args --params-file /home/nvidia/mdl_ws/src/etc/yaml/mdl_dsm.yaml
