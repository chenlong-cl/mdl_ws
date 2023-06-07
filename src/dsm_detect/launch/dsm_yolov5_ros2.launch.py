from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rviz_dir = os.path.join(get_package_share_directory("dsm_cpp2"), "rviz_cfg","dsm_yolov5.rviz")
    rviz_dir = "/home/nvidia/mdl/dsm/yolov5_ws/src/dsm_cpp2/rviz_cfg/dsm_yolov5.rviz"
    rviz_node = Node(
        package='rviz2',
        node_namespace='yolov5',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')
    return LaunchDescription([
        Node(
            package='dsm_pkg',
            namespace='yolov5_pub',
            # node_executable='rtsp_driver_node.py',
            executable='yolov5_pub',
            name='rtsp_camera_driver_node'
        ),
        # Node(
        #     package='dsm_pkg',
        #     namespace='yolov5_json_sub',
        #     # node_executable='rtsp_driver_node.py',
        #     executable='yolov5_json_sub',
        #     name='yolov5_json_sub'
        # ),
        Node(
            package='dsm_pkg',
            namespace='Gpio_read',
            # node_executable='rtsp_driver_node.py',
            executable='Gpio_read',
            name='Gpio_read'
        ),
        Node(
            package='dsm_cpp2',
            executable='dsm_cpp2',
            parameters=[{
                "engine_dir": "/home/nvidia/mdl/dsm/yolov5_ws/src/dsm_cpp2/yolov5_tensorrt_build/best_end.engine",
                'data_type': 'fp16',
            }],
        ),
        # Node(
        #     package='demo_pp_yolov5',
        #     executable='demo_pp_yolov5',
        #     parameters=[{
        #         #'score_threshold': 0.5,
        #         'pointpillar_onnx_file': "/home/nvidia/mdl/dsm/yolov5_ws/src/CUDA-PointPillars-Yolov5/model/pointpillar.onnx", 
        #         "engine_dir": "/home/nvidia/mdl/dsm/yolov5_ws/src/CUDA-PointPillars-Yolov5/model/best.engine",
        #         #'data_type': 'fp16',
        #         #'intensity_scale': 1.0,
        #     }],
        # ),
        rviz_node
    ])

