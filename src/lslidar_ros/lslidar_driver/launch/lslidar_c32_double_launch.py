#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os


def generate_launch_description():
    driver_dir_left = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_c32_1.yaml')
    driver_dir_right = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_c32_2.yaml')

    driver_node_left= LifecycleNode(package='lslidar_driver',
                                namespace='c32_left',
                                executable='lslidar_c32driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir_left],
                                )

    driver_node_right = LifecycleNode(package='lslidar_driver',
                                namespace='c32_right',
                                executable='lslidar_c32driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir_right],
                                )
    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar_c32.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    return LaunchDescription([
        driver_node_left,
        driver_node_right,
        rviz_node
    ])
