source /opt/ros/foxy/setup.bash

###############################################################
###################### R O S l o g ############################
###############################################################
current_time=`date +%Y-%m-%d_%H-%M-%S`
export ROS_LOG_DIR=/home/nvidia/datalog/mdl_ros_logs/${current_time}


###############################################################
###################### F T P Delete ###########################
###############################################################
filepathlist=("/home/nvidia/datalog/dsm/videos" 
	"/home/nvidia/datalog/dsm/images" 
	"/home/nvidia/datalog/dsm/Alarm_logs" 
	"/home/nvidia/datalog/dsm/system_logs"
	"/home/nvidia/datalog/mdl_ros_logs"
)

for filepath in "${filepathlist[@]}"
do
	mkdir -p $filepath
	if [[ $filepath =~ "video" ]]
	then
		find $filepath -mindepth 1 -maxdepth 1 -mtime +5 -exec rm -rf {} \;
	fi
	if [[ $filepath =~ "image" ]]
	then
		find $filepath -mindepth 1 -maxdepth 1 -mtime +28 -exec rm -rf {} \;
	fi
	if [[ $filepath =~ "logs" ]]
	then
		find $filepath -mindepth 1 -maxdepth 1 -mtime +28 -exec rm -rf {} \;
	fi
done


###############################################################
###################### G P I O ################################
###############################################################
sudo chmod 777 /sys/class/gpio/export
sudo echo 350 >/sys/class/gpio/export
sudo chmod 777 /sys/class/gpio/PA.02/value
sudo chmod 777 /sys/class/gpio/PA.02/direction
sudo echo "in" >/sys/class/gpio/PA.02/direction
sudo chmod 777 /sys/class/gpio/export
sudo echo 390 >/sys/class/gpio/export
sudo chmod 777 /sys/class/gpio/PG.07/value
sudo chmod 777 /sys/class/gpio/PG.07/direction
sudo echo "in" >/sys/class/gpio/PG.07/direction


###############################################################
######################   N  T  P   ################################
###############################################################
#sudo ntpdate -u 192.168.1.88
