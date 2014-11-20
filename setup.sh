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

#creates the first part of the roddep
if [[ $o == "-1" || "$#" -eq 0 ]]; then
	echo "creating first part of turtle bot ...."
	if [[ ! -d "rocon" ]]; then
		mkdir rocon
	fi
	cd rocon
	wstool init -j5 src https://raw.github.com/robotics-in-concert/rocon/indigo/rocon.rosinstall

	source /opt/ros/indigo/setup.bash
	rosdep install --from-paths src -i -y

	catkin_make
	cd ..
fi

if [[ $o == "-2" || "$#" -eq 0 ]]; then
	echo "creating second part of turtle bot ...."

	if [[ ! -d "kobuki" ]]; then
		mkdir kobuki
	fi
	cd kobuki
	wstool init src -j5 https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/kobuki.rosinstall
	source ./../rocon/devel/setup.bash
	rosdep install --from-paths src -i -y

	catkin_make
	cd ..
fi

if [[ $o == "-3" || "$#" -eq 0 ]]; then
	echo "creating third part of turtle bot ...."
	if [[ ! -d "turtlebot" ]]; then
		mkdir turtlebot
	fi
	cd turtlebot
	wstool init src -j5 https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/turtlebot.rosinstall
	source ./../kobuki/devel/setup.bash
	rosdep install --from-paths src -i -y -r

	catkin_make
	cd ..
fi

if [[ -d "turtlebot" ]]; then
	cd turtlebot
	if [[ -d "devel" ]]; then
		source ./devel/setup.bash
		rosrun kobuki_ftdi create_udev_rules
	fi
	cd ..
fi