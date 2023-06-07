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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='demo_pp_yolov5',
            executable='demo_pp_yolov5',
            parameters=[{
                'nms_overlap_threshold': 0.01,
                'score_threshold': 0.5,
                "point_sub_topic_name": "/c16/lslidar_point_cloud",
                'pointpillar_onnx_file': "/home/nvidia/mdl_ws/src/adas_detect/model/pointpillar.onnx", 
                "engine_dir": "/home/nvidia/mdl_ws/src/adas_detect/model/best.engine",
                'data_type': 'fp16',
                'intensity_scale': 1.0,
            }],
        ),
       Node(
           package="rviz2",
           executable="rviz2",
           name="rviz2",
           output="screen",
       )
    ])
