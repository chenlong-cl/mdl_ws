#!/bin/bash
export HOME=/home/nvidia
. /opt/ros/foxy/setup.bash
. /home/nvidia/mdl_ws/install/setup.bash
/home/nvidia/mdl_ws/install/can_message/lib/can_message/talker
