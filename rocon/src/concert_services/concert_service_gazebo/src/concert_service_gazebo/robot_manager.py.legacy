#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Abstract base class that defines the API of a robot-specific module needs to
# be defined. This robot specific module will be used by GazeboRobotManager to
# spawn and kill robots in gazebo.

##############################################################################
# Imports
##############################################################################

from abc import ABCMeta, abstractmethod

class RobotManager(object):
    '''
      Abstract base class that defines the API of a robot specific manager.
    '''
    __metaclass__ = ABCMeta

    @abstractmethod
    def spawn_robot(self, name, position_vector):
        """
        Spawn a robot in gazebo with a given name and location.

        :param name str: The robot's name.
        :param position_vector float[]: The location at which the robot needs to
            be spawned. Can be specified as [x,y], [x,y,yaw], or
            [x,y,z,roll,pitch,yaw]. If list length is different from these 3,
            then the behavior is not defined (but an exception is not thrown).
        """
        pass

    @abstractmethod
    def delete_robot(self, name): 
        """
        Delete a previously spawned robot from gazebo.

        :param name str: The robot's name.
        """
        pass

    @abstractmethod
    def prepare_rocon_launch_text(self, robots): 
        """
        Prepare the concert client text for robot(s). This allows you to fine
        tune the concert client that will come up for this robot. A good way of
        doing so is to wrap robot.launch in concert_service_gazebo with a 
        concert launcher with a whitelist of concert names, as well as the 
        packages from which rapps should be loaded into this client. 

        :param robots str[]: Names of robots for whom clients needs to be 
            generated. All the clients can be bundled into a single concert
            launcher
        :return str: The concert launch text (i.e rocon_launch text)
        """
        return ""

    def get_flip_rule_list(self, name):
        """
        A lot of information will have to be flipped from gazebo to each
        concert client, so that the client can truly control the robot. For
        instance, if you wish to simply teleoperate the robot, /cmd_vel and
        /odom should be sufficient. If you want to use full autonomous
        navigation, you'll also have to flip the map, the sensor data, tf
        information and clock.

        :param name str: The name of the robot to whom we are flipping info.
        :return rules gateway_msgs.msg.Rule[]: The rules that need to be flipped
            to the robot's concert client.
        """
        return []
