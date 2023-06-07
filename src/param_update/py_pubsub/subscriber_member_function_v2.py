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
import sys
from multiprocessing import Process
import subprocess
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription1 = self.create_subscription(
            String,
            'param_update_message',
            self.listener_callback,
            10)
        self.subscription1  # prevent unused variable warning
        
        '''self.subscription2 = self.create_subscription(
            String,
            'flag_detection',
            self.detection_callback,
            10)'''
        #self.subscription2  # prevent unused variable warning
        self.publisher_flag = self.create_publisher(String, 'flag_detection', 10)

    def listener_callback(self, msg):
        #pids = psutil.pids()
        #for pid in pids:
        adas_pub_cmd='sudo systemctl stop adas_pub.service'
        adas_laser_pub_cmd='sudo systemctl stop adas_laser_pub.service'
        adas_detect_cmd='sudo systemctl stop adas_detected.service'
        dsm_pub_cmd='sudo systemctl stop dsm_pub.service'
        dsm_detect_cmd='sudo systemctl stop dsm_detected.service'
        subprocess.Popen(adas_pub_cmd,shell=True)
        subprocess.Popen(adas_laser_pub_cmd,shell=True)
        subprocess.Popen(adas_detect_cmd,shell=True)
        subprocess.Popen(dsm_pub_cmd,shell=True)
        subprocess.Popen(dsm_detect_cmd,shell=True)
        adas_yaml=open("/home/nvidia/mdl_ws/src/etc/yaml/mdl_adas_test.yaml","w")
        dsm_yaml=open("/home/nvidia/mdl_ws/src/etc/yaml/mdl_dsm_test.yaml","w")
        param_update_data=eval(msg.data)
        adas_update_date=param_update_data["adas"]
        dsm_update_date=param_update_data["dsm"]
        adas_yaml_data={
        		 'demo_pp_yolov5':
        			{'ros__parameters':
        				adas_update_date
                        }
        	    }
        dsm_yaml_data={
        		'dsm_detect':
        			{'ros__parameters':
        				dsm_update_date
                        }
        	    }
        yaml.dump(adas_yaml_data,adas_yaml)
        yaml.dump(dsm_yaml_data,dsm_yaml)
        adas_yaml.close()
        dsm_yaml.close()
        adas_restart_pub_cmd='sudo systemctl start adas_pub.service'
        adas_restart_laser_pub_cmd='sudo systemctl start adas_laser_pub.service'
        adas_restart_detect_cmd='sudo systemctl start adas_detected.service'
        dsm_restart_pub_cmd='sudo systemctl start dsm_pub.service'  
        dsm_restart_detect_cmd='sudo systemctl start dsm_detected.service' 
        subprocess.Popen(adas_restart_pub_cmd,shell=True)
        subprocess.Popen(adas_restart_laser_pub_cmd,shell=True)
        subprocess.Popen(adas_restart_detect_cmd,shell=True)
        subprocess.Popen(dsm_restart_pub_cmd,shell=True)
        subprocess.Popen(dsm_restart_detect_cmd,shell=True)
        self.get_logger().info('I heard: "%s"' % msg.data)

    def detection_callback(self, msg):
        if msg.data == "1":
            #cmd='ros2 launch demo_pp_yolov5 pp_yolov5_infer_launch.py'
            #os.system(cmd)
            adas_cmd='sudo systemctl restart adas_detected.service'
            dsm_cmd='sudo systemctl restart dsm_detected.service'
            subprocess.Popen(adas_cmd,shell=True)
            subprocess.Popen(dsm_cmd,shell=True)
        else:
            pass
def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    #listen_process=Process(target=listener_callback)
    #detection_process=Process(target=detection_callback)
    #listen_process.start()
    #detection_process.start()
    
