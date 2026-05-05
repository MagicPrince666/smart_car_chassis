#!/bin/bash

source /opt/ros/foxy/setup.bash
source /home/tspi/smart_car_chassis/install/setup.bash

echo "admin1234." | sudo -S /home/tspi/smart_car_chassis/src/chassis/scripts/tspi_two_wheel_setup.sh

processNum=`ps -ef | grep chassis_node | grep -v grep | wc -l`

if [ $processNum -eq 0 ]; then
	echo "chassis start"
	ros2 launch chassis tspi_ldlidar.launch.py &
else
	echo "chassis running"
fi

