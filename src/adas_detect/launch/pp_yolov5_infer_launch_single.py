# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import time

date=time.strftime('%Y-%m-%d',time.localtime())
time=time.strftime('%H:%M:%S',time.localtime())
def generate_launch_description():

    #pkg_share_dir = get_package_share_directory('demo_pp_yolov5')
    #yaml_path = pkg_share_dir + '/configs/param.yaml'
    yaml_path=os.path.join(get_package_share_directory('demo_pp_yolov5'), 'configs', 'param.yaml')
    return LaunchDescription([
        Node(
            package='demo_pp_yolov5',
            executable='demo_pp_yolov5',
            name='demo_pp_yolov5',
            parameters=[{
                "point_sub_topic_name": "/c16/lslidar_point_cloud",
                "image_sub_topic_name": "/adas/video_frames",
                'pointpillar_onnx_file': "/home/nvidia/mdl_ws/src/etc/mdl_adas_model/pointpillar.onnx", 
                "engine_dir": "/home/nvidia/mdl_ws/src/etc/mdl_adas_model/mdl_0528.engine",
                'date': date,
                'time': time,
            },
            	yaml_path,
            ],
        ),
       Node(
           package="rviz2",
           executable="rviz2",
           name="rviz2",
           output="screen",
       )
    ])
