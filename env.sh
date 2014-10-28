#! /bin/bash

echo "See .env.log for any potential errors"

cd rocon/; catkin_make; cd .. # >> .env.log
source ./rocon/devel/setup.bash # >> .env.log
cd kobuki/; catkin_make; cd .. #>> .env.log
source ./kobuki/devel/setup.bash #>> .env.log
cd turtlebot/; catkin_make; cd .. # >> .env.log
source ./turtlebot/devel/setup.bash # >> .env.log
