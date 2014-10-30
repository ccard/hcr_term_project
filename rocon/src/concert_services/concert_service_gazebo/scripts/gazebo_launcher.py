#!/usr/bin/env python

import os
import concert_service_utilities
import rospy
import rocon_python_utils

from concert_service_gazebo import GazeboRobotManager

def loginfo(msg):
    rospy.loginfo('Gazebo Launcher : %s'%str(msg))

if __name__ == '__main__':
    rospy.init_node('gazebo_launcher')
    (service_name, unused_service_description, unused_service_id, unused_key) = concert_service_utilities.get_service_info()
    world_file = rospy.get_param('world_file', [])
    world_name = rospy.get_param('~world', 'world')
    gazebo_binary = rospy.get_param('~gazebo_binary', 'gzserver')
    env = os.environ.copy()
    
    world_file_unpacked = rocon_python_utils.ros.find_resource_from_string(world_file)

    loginfo('Start')
    loginfo('World File : %s'%world_file_unpacked) 
    loginfo('World Name : %s'%world_name) 
    loginfo('Binary     : %s'%gazebo_binary) 

    command_args = ['rosrun', 'gazebo_ros', gazebo_binary, world_file_unpacked, '_name:=' + world_name]
    loginfo(str(command_args))
    process = rocon_python_utils.system.Popen(command_args, env=env)
    rospy.spin()
    process.terminate()
    loginfo('ByeBye')
