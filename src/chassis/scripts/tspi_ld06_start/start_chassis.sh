#!/bin/bash

source /opt/ros/galactic/setup.bash
source /home/i6sp/smart_car_chassis/install/setup.bash

echo "i2ctech" | sudo -S /home/i6sp/smart_car_chassis/src/chassis/scripts/luban_cat_two_wheel_setup.sh

processNum=`ps -ef | grep chassis_node | grep -v grep | wc -l`

if [ $processNum -eq 0 ]; then
	echo "chassis start"
	ros2 launch chassis tspi_ldlidar.launch.py &
else
	echo "chassis running"
fi

