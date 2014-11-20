#! /bin/bash

# Author: Chris Card, Marshall Sweatt
# This file launches the turtlebot gmapping and visulization of
# the mapping process

#--------------------------------------------------------------------
# Global variables
roscmd="roslaunch"

turtlebot="turtlebot_navigation gmapping_demo.launch"
rviz="turtlebot_rviz_launchers view_navigation.launch"

logdir="logs"

gmapping_std="${logdir}/gmapping_std.log"
gmapping_err="${logdir}/gmapping_err.log"

mapping_rviz_std="${logdir}/mapping_rviz_std.log"
mapping_rviz_err="${logdir}/mapping_rviz_err.log"

gmapping_log_options="> ${gmapping_std} 2> ${gmapping_err}"
rviz_log_options="> ${mapping_rviz_std} 2> ${mapping_rviz_err}"

gmapping_cmd="${roscmd} ${turtlebot} ${gmapping_log_options}"

rviz_cmd="${roscmd} ${rviz} ${rviz_log_options}"

if [[ ! -d $logdir ]]; then
	echo -n "Creating logs dir .... "
	mkdir $logdir
	echo "done"
fi

#--------------------------------------------------------------------
# Functions for launching kobuki commands
function turtlebringup {
	eval $1
}

function keyopbringup {
	eval $1
}

function endros {
	local __resvar=$1
	read -p "Quit [y or n]:" -n 1 -r
	echo
	if [[ $REPLY =~ ^[Yy]$ ]]; then
		eval $__resvar="'y'"
	else
		eval $__resvar="'n'"
	fi
}

function usage {
	echo "./gmapping_launch [-s]"
	echo " 	-s launches the screen"
	echo "  -h this message"
	exit
}

#--------------------------------------------------------------------
# Execution phase

if [[ $# >= 1 ]]; then
	if [[ $1 -eq "-h" ]]; then
		usage
	elif [[ $# > 1 || $1 -ne "-s" ]]; then
		usage
	fi
fi

echo "For error information please see ${gmapping_std}, "
echo "${gmapping_err}, ${mapping_rviz_std}, and ${mapping_rviz_err}"

echo "Bringingup Turtlebot .... "
turtlebringup "${gmapping_cmd}" & pid_gmapping=$!

sleep 10

pid_rviz=-1

if [[ $# == 1 ]]; then
	if [[ $1 -eq "-s" ]]; then
		echo "Bringingup keyop .... "
		keyopbringup "${rviz_cmd}" & pid_rviz=$!
	else
		echo "Invalid option"
	fi
fi

sleep 10

echo "Everything should be ready to run .... "

endros result

while [[ $result != "y" ]]; do
	endros result
done

echo "Terminating processes .... "
kill $pid_rviz
kill $pid_gmapping

echo "Killing the straglers .... "
`pkill -f ${roscmd}`

echo "All done have a nice day!"
