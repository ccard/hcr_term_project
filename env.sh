#! /bin/bash

cd rocon/; catkin_make; cd ..
source /rocon/devel/setup.bash
cd kobuki/; catkin_make; cd ..
source /kobuki/devel/setup.bash
cd turtlebot/; catkin_make; cd ..
source /turtle/devel/setup.bash
