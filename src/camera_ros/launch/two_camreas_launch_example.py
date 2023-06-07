#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
import subprocess


def generate_launch_description():
    driver_params_dir1 = os.path.join(get_package_share_directory('camera_ros'), 'params', 'camera_ros_adas.yaml')
    driver_params_dir2 = os.path.join(get_package_share_directory('camera_ros'), 'params', 'camera_ros_dsm.yaml')
    rviz_dir = os.path.join(get_package_share_directory('camera_ros'), 'rviz_cfg', 'adas_dsm_camera_ros.rviz')


    driver_dsm_node = LifecycleNode(package='camera_ros',
                                    namespace='dsm',
                                    executable='camera_ros_node',
                                    name='camera_ros_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[driver_params_dir2],
                                    )
    driver_adas_node = LifecycleNode(package='camera_ros',
                                    namespace='adas',
                                    executable='camera_ros_node',
                                    name='camera_ros_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[driver_params_dir1],
                                    )
    rviz_node = Node(
            package='rviz2',
            namespace='camera_ros',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')



    return LaunchDescription([
        driver_dsm_node, driver_adas_node, rviz_node
    ])