#!/usr/bin/env python

import concert_service_utilities
import rospy

from concert_service_gazebo import GazeboRobotManager

if __name__ == '__main__':
    rospy.init_node('gazebo_robot_manager')
    (service_name, unused_service_description, unused_service_id, unused_key) = concert_service_utilities.get_service_info()
    robots      = rospy.get_param('robots', [])
    #robot_types = rospy.get_param('types', [])
    world_name = rospy.get_param('world', 'gazebo')
    world_namespace = '/services/' + service_name + '/' + str(world_name) + '/'
    concert_name = rospy.get_param('/concert/name')
    gazebo_manager = GazeboRobotManager(world_namespace, concert_name)
    gazebo_manager.loginfo('spawning robots : %s'%str(robots))
    gazebo_manager.spawn_robots(robots)

    gazebo_manager.spin()
    gazebo_manager.loginfo('Bye Bye')
