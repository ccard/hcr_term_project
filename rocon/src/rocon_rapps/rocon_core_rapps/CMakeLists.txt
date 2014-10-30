##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_core_rapps)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY chirp DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY listener DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY talker DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY teleop DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
