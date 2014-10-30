##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(turtle_rapps)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY turtle_stroll DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY teleop DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
