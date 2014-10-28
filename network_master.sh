#! /bin/bash

if [[ "$#" -ne 1 ]]; then
	echo "source network_master.sh <turtle_ip>"
	echo "  <turtle_ip> = ip address of turtlebot"
else
	export ROS_MASTER_URI=http://$1:11311
	export ROS_HOSTNAME=`ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`
fi
