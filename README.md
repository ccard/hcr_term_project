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
&nbsp;&nbsp;This README describes how to use our program to generate SLAM based on gmapping and then use the generated map to autonomusly navigate through the mapped environment. This rest of this file will explain how to setup the [environment](#environment), how to [compile](#compilation) our program and how to [run](#execution) gmapping and the autonomus navigation.

---------------
# Environment #
&nbsp;&nbsp;This section describes how to setup the environment on the turtlebot and workstation.  It also describes the expected environment that it will be run in. __Note:__ These steps must be repeated for every new terminal opened.

## Expected environment ##
 - Ubuntu 14.04 (__!!Not Guaranteed to work with out this!!__)
 - Turtlebot with connect and kobuki base
 - ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) (__!!Not Guaranteed to work with out this!!__)
 - Run `source setup.sh` to get all the necessary files for the turtlebots. This script performs the setup for the Turtlebot ROS packages as described in the [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation) (__note__ if you are having trouble installing turtlebot packages please follow the tutorial)
 - The command `source /opt/ros/indigo/setup.bash` has been run
 - the ncurses library is installed to do so run `sudo apt-get install libncurses5-dev`
 - The hector slam  program `sudo apt-get install ros-indigo-slam-gmapping`
 - If having trouble getting kinect to work on ubuntu 14.04 follow this [post](https://github.com/OpenPTrack/open_ptrack/issues/19) specifically the post:
 
> I've just found the issue: the latest Ubuntu driver is not working.
> I solved it by downloading older versions of the drivers ([libopenni-sensor-primesense-dev](https://launchpad.net/%7Ev-launchpad-jochen-sprickerhof-de/+archive/ubuntu/pcl/+build/5252450/+files/libopenni-sensor-primesense-dev_5.1.0.41-3%2Btrusty1_amd64.deb) and [libopenni-sensor-primesense0](https://launchpad.net/%7Ev-launchpad-jochen-sprickerhof-de/+archive/ubuntu/pcl/+build/5252450/+files/libopenni-sensor-primesense0_5.1.0.41-3%2Btrusty1_amd64.deb)) and installing them manually:
> ```cd ~/Downloads
sudo dpkg -i libopenni-sensor-primesense*```
>Hope it works also for you!


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
&nbsp;&nbsp;The program will be compiled when the commmand `source env.sh`

---------------
# Execution #

## Turtlebot Bringup ##
### Creating SLAM map ###
&nbsp;&nbsp;&nbsp; This section describes how to execute the project on the turlebot. Please follow these steps in order and open 3 terminals using the env setup above:
- In terminal 1 `./kobuki_bringup.sh` to prep the turtlebot for remote telleoperation for movement controll
- In terminal 2 run `./gmapping_bringup.sh` add `-s` if you want the rviz terminal.
- When finished mapping (___Do not stop any of the prgrams___) in the third terminal run the command `rosrun map_server map_saver -f /tmp/my_map` to save the slam map once this is done the programs can be terminated

### Running Autonomously ###
Follow this [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)
 - Must have both `<file_name>.pgm` and `<file_name>.yaml` in tmp folder.
 - in one terminal run `./autonomus_navigation.sh /tmp/<file_name>.yaml`
 - in the second terminal run `roslaunch turtlebot_rviz_launchers view_navigation.launch --screen`

## Workstation ##
&nbsp;&nbsp;&nbsp; This section describes how to execute the on the work station after the above [turtlebot bringup](#turtlebot_bringup) and environment setup has been completed: run `rosrun core keyop_controller_publisher` this will allow you to remote control the turtle bot.

# Results #

&nbsp;&nbsp;__Demo Video__(If not working click [here](http://www.youtube.com/watch?v=1JjsBqtKpcY&list=UUcS7AZZsCauWAlKDbMXsKOw))
<iframe width="560" height="315" src="//www.youtube.com/embed/1JjsBqtKpcY?list=UUcS7AZZsCauWAlKDbMXsKOw" frameborder="0" allowfullscreen></iframe>

# References #
- ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- ROS Indigo turtlebot instilation [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation)
- Ros gmapping recoding [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM)
- Ros autonomus navigation [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)


# Appendix #


--------------------------------------------------------------------------------
# Notes (please disregarad these as they are used only as reminders to ourselves please follow the instructions above) #

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

## Running Autonomously? ##
Follow this [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)
Must have both `<file_name>.pgm` and `<file_name>.yaml` in tmp folder.
in one terminal run `./kobuki_bringup.sh`
in second terminal run `roslaunch core generateOdomAndSlam.launch map_file:=/tmp/<file_name>.yaml`
in fourth terminal run `roslaunch turtlebot_rviz_launchers view_navigation.launch --screen`
~~in second terminal run `roslaunch core generateOdomAndSlam.launch'~~
~~in third terminal run `roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/<file_name>.yaml`~~