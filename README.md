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
&nbsp;&nbsp;This section describes how to setup the environment on the turtlebot and workstation.  It also describes the expected environment that it will be run in.

## Expected environment ##
 - Ubuntu 14.04 or higher
 - ROS [Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
 - The command `source /opt/ros/indigo/setup.bash` has been run

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

# how to control kobuki remotely from topics #
publish kobuki_msgs/KeyboardInput object to  /keyop/teleop when running kobuki_keyop keyop.launch to control kobuki base