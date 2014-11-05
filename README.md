HCR Term Project
================
__Authors__: Chris Card, Marshall Sweatt

# Introduction #

---------------
# Environment #
&nbsp;&nbsp;This section describes how to setup the environment on the turtlebot and workstation.  It also describes the expected environment that it will be run in.

## Expected environment ##
 - Ubuntu 14.04 or higher
 - ROS Indigo
 - The command `source /opt/ros/indigo/setup.bash` has been run

## Workstation ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the workstation
 - Run `source env.sh` from the project directory
 - Run `source network_master.sh <turtlebot ip>` from the project directory with the turtlbots ip address passed in

## Turtlebot ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the turtlebot
 - Run `source env.sh` from the project directory
 - Run `source network_kobuki.sh` from the project directory
 - Run `./kobuki_bringup.sh` to prep the turtlebot for remote telleoperation

---------------
# Compilation #

---------------
# Execution #



# how to control kobuki remotely from topics #
publish kobuki_msgs/KeyboardInput object to  /keyop/teleop when running kobuki_keyop keyop.launch to control kobuki base