#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/orangepi/linux-car/install/setup.bash

echo "orangepi" | sudo -S /home/orangepi/linux-car/src/chassis/scripts/mqquad_two_wheel_setup.sh

processNum=`ps -ef | grep chassis_node | grep -v grep | wc -l`

if [ $processNum -eq 0 ]; then
	echo "chassis start"
	ros2 launch chassis chassis_ydlidar.launch.py &
else
	echo "chassis running"
fi

