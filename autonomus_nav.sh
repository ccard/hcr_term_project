#! /bin/bash

# Author: Chris Card, Marshall Sweatt
# This file launches the autonmus navigation launch and keyop bringup
# which is responsible for control of the turtlebot

#--------------------------------------------------------------------
# Global variables
roscmd="roslaunch"

turtlebot="core generateOdomAndSlam.launch"
keyop="kobuki_keyop keyop.launch"

logdir="logs"

turtlebot_std="${logdir}/turtlebot_std.log"
turtlebot_err="${logdir}/turtlebot_err.log"

kobuki_std="${logdir}/kobuki_std.log"
kobuki_err="${logdir}/kobuki_err.log"

turtlebot_log_options="> ${turtlebot_std} 2> ${turtlebot_err}"
kobuki_log_options="> ${kobuki_std} 2> ${kobuki_err}"

turtlebot_cmd="${roscmd} ${turtlebot} ${turtlebot_log_options}"

kobuki_cmd="${roscmd} ${keyop} ${kobuki_log_options}"

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
	echo "autonomus_nav.sh /tmp/<map_file>.yaml"
	echo "  The <map_file>.yaml and <map_file>.pmg must be in the tmp directory"
	echo "  so the ros program can access it."
	exit
}

#--------------------------------------------------------------------
# Execution phase

if [[ $# != 1 ]]; then
	usage
else

map_file=$1

turtlebot_cmd="${roscmd} ${turtlebot} map_file:=${map_file} ${turtlebot_log_options}"

echo "For error information please see ${turtlebot_std}, "
echo "${turtlebot_err}, ${kobuki_std}, and ${kobuki_err}"

echo "Bringingup Turtlebot .... "
turtlebringup "${turtlebot_cmd}" & pid_turtle=$!

sleep 10

echo "Bringingup keyop .... "
keyopbringup "${kobuki_cmd}" & pid_keyop=$!

sleep 10

echo "Everything should be ready to run .... "

endros result

while [[ $result != "y" ]]; do
	endros result
done

echo "Terminating processes .... "
kill $pid_keyop
kill $pid_turtle

echo "Killing the straglers .... "
`pkill -f ${roscmd}`

echo "All done have a nice day!"
fi