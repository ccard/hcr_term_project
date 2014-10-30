##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(kobuki_rapps)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY auto_docking DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY random_walker DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
