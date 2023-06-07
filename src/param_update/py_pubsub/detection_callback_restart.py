# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import String
import os
import psutil
import signal
from multiprocessing import Process
import subprocess
class MinimalRestarter(Node):

    def __init__(self):
        super().__init__('minimal_restart')
        
        self.subscription2 = self.create_subscription(
            String,
            'flag_detection',
            self.detection_callback,
            10)
        self.subscription2  # prevent unused variable warning

    def detection_callback(self, msg):
        if msg.data == "1":
            cmd='ros2 launch demo_pp_yolov5 pp_yolov5_infer_launch.py'
            #os.system(cmd)
            subprocess.Popen(cmd,shell=True)

        else:
            pass
def main(args=None):
    
    rclpy.init(args=args)

    minimal_restarer = MinimalRestarter()

    rclpy.spin(minimal_restarer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_restarer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    #listen_process=Process(target=listener_callback)
    #detection_process=Process(target=detection_callback)
    #listen_process.start()
    #detection_process.start()
    
