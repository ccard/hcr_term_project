#! /bin/bash

function usage {
	echo "setup.sh [option]"
	echo "[option] = stage of the program to execute blank for all"
	echo "  -1 => first part"
	echo "  -2 => second part"
	echo "  -3 => third part"
	echo "  -h => this message"
	exit
}
o=""
if [[ "$#" -eq 1 ]]; then
	o="$1"
	if [[ $1 == "-h" ]]; then
		usage
	fi
fi

if [[ ! -d "turtlebot_arm" ]]; then
	mkdir turtlebot_arm
fi

cd turtlebot_arm

wstool init src
cd src
wstool set turtlebot_arm https://github.com/turtlebot/turtlebot_arm.git --git --version=indigo-devel
wstool update turtlebot_arm
cd ..
source /opt/ros/indigo/setup.bash
rosdep install --from-paths src -i -y
catkin_make

