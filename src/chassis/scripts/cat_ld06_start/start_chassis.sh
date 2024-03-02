#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/cat/linux-car/install/setup.bash

echo "temppwd" | sudo -S /home/cat/linux-car/src/chassis/scripts/luban_cat_ackerman_setup.sh

processNum=`ps -ef | grep chassis_node | grep -v grep | wc -l`

if [ $processNum -eq 0 ]; then
	echo "chassis start"
	ros2 launch chassis ackerman_chassis_ld06.launch.py &
else
	echo "chassis running"
fi

