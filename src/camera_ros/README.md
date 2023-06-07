### Camera_ROS2驱动说明

## 1.工程介绍

​		Camera_ROS2为linux环境下RTSP camera ros2驱动，适用于RTSP url,device 0(need to fix encoding format (8UC3, bgr8)) etc，程序在ubuntu 20.04 ros foxy下测试通过。

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.依赖
Opencv

3.其他依赖

~~~bash
sudo apt install libspdlog-dev
~~~

## 3.编译与运行：

~~~shell
mkdir -p ~/foxy_ws/src
cd ~/foxy_ws/src
拷贝到src目录
cd ~/foxy_ws
colcon build
source install/setup.bash
#fix the device ip
ros2 run camera_ros camera_ros_node --ros-args -r __node:=camera_ros_node -r __ns:=/dsm --params-file src/camera_ros/params/camera_ros_dsm.yaml
#fix the device ip
ros2 launch camera_ros two_camreas_launch_example.py


