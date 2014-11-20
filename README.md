HCR Term Project
================
__Authors__: Chris Card, Marshall Sweatt

To see the nicely formated version of this file go [here](https://github.com/ccard/hcr_term_project/blob/master/README.md)

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
&nbsp;&nbsp;This README describes how to use our program that generates a SLAM based on gmapping.  This approach is then used to generate a map for autonomusly navigation through the mapped environment. This rest of this file will explain how to setup the [environment](#environment), how to [compile](#compilation) our program and how to [run](#execution) gmapping and the autonomus navigation.

---------------
# Environment #
&nbsp;&nbsp;This section describes how to setup the environment on the turtlebot and workstation.  It also describes the expected environment that it will be run in. __Note:__ These steps must be repeated for every new terminal opened.

## Expected environment ##
 - Ubuntu 14.04 (__!!Not Guaranteed to work with out this!!__)
 - Turtlebot with Kinect and Kobuki base
 - ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) (__!!Not Guaranteed to work with out this!!__)
 - Run `source setup.sh` to get all the necessary files for the turtlebots. This script performs the setup for the Turtlebot ROS packages as described in the [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation) (__note__ if you are having trouble installing turtlebot packages please follow the tutorial)
 - The command `source /opt/ros/indigo/setup.bash` has been run
 - the ncurses library (used as teleop control interface) is installed. To install run `sudo apt-get install libncurses5-dev`
 - The gmapping slam program `sudo apt-get install ros-indigo-slam-gmapping`
 
__Note__: If having trouble getting kinect to work on ubuntu 14.04 follow this [post](https://github.com/OpenPTrack/open_ptrack/issues/19) specifically the post:
> I've just found the issue: the latest Ubuntu driver is not working.<br>
> I solved it by downloading older versions of the drivers ([libopenni-sensor-primesense-dev](https://launchpad.net/%7Ev-launchpad-jochen-sprickerhof-de/+archive/ubuntu/pcl/+build/5252450/+files/libopenni-sensor-primesense-dev_5.1.0.41-3%2Btrusty1_amd64.deb) and [libopenni-sensor-primesense0](https://launchpad.net/%7Ev-launchpad-jochen-sprickerhof-de/+archive/ubuntu/pcl/+build/5252450/+files/libopenni-sensor-primesense0_5.1.0.41-3%2Btrusty1_amd64.deb)) and installing them manually:<br>
> `cd ~/Downloads` <br>
> `sudo dpkg -i libopenni-sensor-primesense*`<br>
>Hope it works also for you!


## Workstation ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the workstation (for each terminal)
 - Run `source env.sh` from the project directory
 - Run `source network_master.sh <turtlebot ip>` from the project directory with the turtlbots ip address passed in

## Turtlebot ##
&nbsp;&nbsp;&nbsp;To perform the environment setup of the turtlebot (for each terminal)
 - Run `source env.sh` from the project directory
 - Run `source network_kobuki.sh` from the project directory


---------------
# Compilation #
&nbsp;&nbsp;The program will be compiled when the commmand `source env.sh` is executed. `source env.sh` compiles and builds the `rocon/`,`kobuki/`, `turtlebot/` and `slam_eval_code/` directories.

---------------
# Execution #
&nbsp;&nbsp;Describes how to run the programs on the robot and master computers.

## Turtlebot Bringup ##
&nbsp;&nbsp;This section describes how to bringup the turtlebot for the different tasks on the computer connected to the turtlebot. 

### Creating SLAM map ###
&nbsp;&nbsp;&nbsp; This section describes how to execute the project on the turlebot. Please follow these steps in order and open 3 terminals using the env setup above:
- In terminal 1 `./kobuki_bringup.sh` to prep the turtlebot for remote telleoperation for movement control
- In terminal 2 run `./gmapping_bringup.sh` add `-s` if you want the rviz terminal (We do not use the `-s` option during our testing, but is useful for debugging).
- When finished mapping (___!!Do not stop any of the prgrams!!___) in the third terminal run the command `rosrun map_server map_saver -f <path>/<file_name>` to save the slam map.  We typically save into our current directory by running `rosrun map_server map saver -f <file_name>`. Once this is done the programs can be terminated.

### Running Autonomously ###
&nbsp;&nbsp;&nbsp;The autonomus running scripts of our robot are based on this [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map). If you are having difficulty getting our scripts below to work follow the tutorial. (remember to follow the turtlebot section in the previous environment section for each terminal)
 - Must have both `<file_name>.pgm` and `<file_name>.yaml` in `/tmp` folder.
 - In the first terminal run `./autonomus_navigation.sh /tmp/<file_name>.yaml`
 - In the second terminal run `roslaunch turtlebot_rviz_launchers view_navigation.launch --screen`
 - Once rviz is running, click on the interact button. Then while holding the right mouse button zoom out.
  - Then click 2d pose estimite and click and hold where the robot is approximently and drag the arrow in the direction that the robot is facing then release
  - Once the black dot and the green arrow cloud is where the robot is, click on 2d nav point. Click and hold where you want the robot to go and drag the green arrow in the direction you want the robot's final orientation to be and then release the mouse and the robot will start moving. (_note_: it should not run into walls but if it does it will stop). 

## Workstation ##
&nbsp;&nbsp;&nbsp; This section describes how to execute the on the work station after the above [turtlebot bringup](#turtlebot_bringup) and environment setup have been completed: run `rosrun core keyop_controller_publisher` this will allow you to remote control the turtle bot. If you want to control the turtlebot from the turtlebot computer just simply run the command from a sourced terminal.

# Results #

&nbsp;&nbsp;__Demo Video__(If not working click [here](http://www.youtube.com/watch?v=1JjsBqtKpcY&list=UUcS7AZZsCauWAlKDbMXsKOw))
<iframe width="560" height="315" src="//www.youtube.com/embed/1JjsBqtKpcY?list=UUcS7AZZsCauWAlKDbMXsKOw" frameborder="0" allowfullscreen></iframe>

# References #
- ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- ROS Indigo turtlebot instilation [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation)
- Ros gmapping recoding [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM)
- Ros autonomus navigation [tutorial](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)
- For trouble with kinect drivers look at this [post](https://github.com/OpenPTrack/open_ptrack/issues/19)


# Appendix #
 - All test SLAM files are in `./hallway_results/` or `./Test/`.  If you would like  to test autonomous navigation using one of our premade maps, we suggest you use `./Test/hall_all_1`.
 - `/slam_eval_code/src/core` contains all of the code we wrote and launch files.  The only launch file that we use is `generateOdomAndSlam.launch`, the rest are for testing use.
 - `/docs/` contain our termproject paper (not yet complete)
 - `./autonomus_nav.sh /tmp/<file_name>.yaml` is the command to start the keyop controller and autonomus navigation stack with the map passed in. Please refer to the above sections for instructions on how to start.
 - `./gmapping_bringup.sh [-s]` (`[-s]` is optional as it opens rviz which is not needed to create the map).  When run in conjunction with `./kobuki_bringup.sh` it creates a map. Please refer to the above sections for instructions on how to start.
 - `source network_kobuki.sh` and `source network_master.sh <kobuki ip address>` automatically sets the `$ROS_MASTER_URI` and `$ROS_HOSTNAME` environmental variables to the correct values on the respective machines to establish communication between the robot and workstation.
 - `source env.sh` builds and sources `/rocon/`,`/kobuki/`, `/turtlebot/` and `/slam_eval_code/` if the envorinment for the turtlebot packages was setup correctly.  See the environment [section](#environment) to do this.
 - `source setup.sh` will download and install all the necessary turtlebot packages form the referenced tutorial.

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