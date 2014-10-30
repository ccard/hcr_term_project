##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_service_admin)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

##############################################################################
# Installs
##############################################################################

#install(
#    PROGRAMS 
#        scripts/rocon_interactions
#        scripts/rocon_interactions_manager.py
#        scripts/load_roles_and_apps
#        scripts/attach_test_remocon
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )
#
install(DIRECTORY services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
