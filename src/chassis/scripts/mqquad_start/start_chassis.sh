#!/bin/bash

source /opt/ros/foxy/setup.bash
source /home/orangepi/smart_car_chassis/install/setup.bash

echo "orangepi" | sudo -S /home/orangepi/smart_car_chassis/src/chassis/scripts/mqquad_two_wheel_setup.sh

processNum=`ps -ef | grep chassis_node | grep -v grep | wc -l`

if [ $processNum -eq 0 ]; then
	echo "chassis start"
	ros2 launch chassis chassis_ydlidar.launch.py &
else
	echo "chassis running"
fi

