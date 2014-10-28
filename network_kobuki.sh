#! /bin/bash

export ROS_MASTER_URI=http://`ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`:11311
export ROS_HOSTNAME=`ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`
