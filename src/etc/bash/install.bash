sudo cp mdletc.sh /etc/profile.d/
sudo cp dsm_detected.service /etc/systemd/system/
sudo cp dsm_pub.service /etc/systemd/system/

sudo cp adas_detected.service /etc/systemd/system/
sudo cp adas_laser_pub.service /etc/systemd/system/
sudo cp adas_pub.service /etc/systemd/system/

sudo cp canUp.service /etc/systemd/system/
sudo cp can_messags_talker.service /etc/systemd/system/

sudo systemctl daemon-reload 
sudo systemctl enable dsm_detected.service 
sudo systemctl enable dsm_pub.service

sudo systemctl enable adas_detected.service
sudo systemctl enable adas_laser_pub.service
sudo systemctl enable adas_pub.service
sudo systemctl enable canUp.service
sudo systemctl enable can_messags_talker.service
#mkdir /home/nvidia/datalog/mdl_ros_logs
cd /home/nvidia/mdl_ws/
#colcon build
source /home/nvidia/mdl_ws/install/setup.bash
