HCR Term Project
================
__Authors__: Chris Card, Marshall Sweatt

# Table of Contents #
 - [Introduction](#introduction)
 - [Environment](#environment)
 - [Compilation](#compilation)
 - [Execution](#execution)
 - [Results](#results)
 - [References](#references)
 - [Appendix](#appendix)

----------------
# Introduction #

---------------
# Environment #
&nbsp;&nbsp;This section describes how to setup the environment on the turtlebot and workstation.  It also describes the expected environment that it will be run in. __Note:__ These steps must be repeated for every new terminal opened.

## Expected environment ##
 - Ubuntu 14.04 or higher
 - Turtlebot with connect and kobuki base
 - ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
 - The Turtlebot ROS packages as installed from the [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation)
 - The command `source /opt/ros/indigo/setup.bash` has been run
 - the ncurses library is installed to do so run `sudo apt-get install libncurses5-dev`

## Workstation ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the workstation
 - Run `source env.sh` from the project directory
 - Run `source network_master.sh <turtlebot ip>` from the project directory with the turtlbots ip address passed in

## Turtlebot ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the turtlebot
 - Run `source env.sh` from the project directory
 - Run `source network_kobuki.sh` from the project directory


---------------
# Compilation #

---------------
# Execution #

## Turtlebot ##
&nbsp;&nbsp;&nbsp; This section describes how to execuite the project on the turlebot. Please follow these steps in order:
- Run `./kobuki_bringup.sh` to prep the turtlebot for remote telleoperation for movement controll
- Run `blah` to bring up our slam program

# Results #

# References #

# Appendix #

# Notes #

# usage notes: #

## how to check if kinect is working ##
first run this command `roslaunch turtlebot_bringup 3dsensor.launch`
Use this command to check if kinect is working `rosrun image_view disparity_view image:=/camera/depth_registered/disparity`
if using ubuntu 14 indigo not working then follow this [post](https://github.com/OpenPTrack/open_ptrack/issues/19)

## how to control kobuki remotely from topics ##

publish kobuki_msgs/KeyboardInput object to  /keyop/teleop when running kobuki_keyop keyop.launch to control kobuki base

all slam project topics begin with `slam_kobuki` i.e. `slam_kobuki/keyop`

## how to get gmapping to work :) ##
follow this [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM)
in one terminal run `./kobuki_bringup.sh`
in a seperate terminal run `roslaunch turtlebot_navigation gmapping_demo.launch`
in a seperate terminal run `roslaunch turtlebot_rviz_launch view_navigation.launch`
in a seperate terminal run `rosrun core keyop_controller_publisher`