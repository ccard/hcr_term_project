#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out make a map operations for rocon interactions.
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_uri
import concert_service_utilities
import unique_id
import rocon_std_msgs.msg as rocon_std_msgs
import scheduler_msgs.msg as scheduler_msgs
import concert_service_msgs.msg as concert_service_msgs

class MakeAMapPimp(concert_service_utilities.ResourcePimp):

    _default_cmd_vel_topic = '/teleop/cmd_vel'
    _default_compressed_image_topic = '/teleop/compressed_image'
    _default_map_topic = 'map'
    _default_scan_topic = '/make_a_map/scan'
    _default_robot_pose_topic = 'robot_pose'
    _default_wc_namespace = 'world_canvas'

    def setup_variables(self):
        '''
            Need to setup the following variables
            service_priority, service_id, resource_type, available_resource_publisher_name, capture_topic_name
        '''
        (service_name, service_description, service_priority, service_id) = concert_service_utilities.get_service_info()
        self.service_priority = service_priority
        self.service_id = service_id
        self.resource_type = 'turtlebot_rapps/make_a_map'
        self.available_resource_publisher_name = 'available_make_a_map'
        self.capture_topic_name = 'capture_make_a_map'

    def ros_capture_callback(self, request_id, msg):
        '''
         Processes the service pair server 'capture_teleop'. This will run
         in a thread of its own for each request. It has a significantly long lock
         though - this needs to get fixed.
        '''
        # Todo : request the scheduler for this resource,
        # use self.allocation_timeout to fail gracefully

        response = concert_service_msgs.CaptureResourceResponse()
        response.result = False
        if not msg.release:  # i.e. we're capturing:
            if msg.rocon_uri not in [r.uri for r in self.available_resources]:
                self.logwarn("couldn't capture resource [not available][%s]" % msg.rocon_uri)
                response.result = False
            else:
                resource = self._create_resource(msg.rocon_uri)
                request_result, resource_request_id = self.send_allocation_request(resource)
                response.result = request_result
                if request_result == False:
                    self.logwarn("couldn't capture resource [timed out][%s]" % msg.rocon_uri)
                else:
                    self.loginfo("captured resource [%s][%s]" % (msg.rocon_uri, resource_request_id))
                    response.remappings = resource.remappings
        else:  # we're releasing
            self.send_releasing_request(msg.rocon_uri)
            response.result = True
        return response

    def _create_resource(self, uri):
        # Create a resource to request
        resource = scheduler_msgs.Resource()
        resource.id = unique_id.toMsg(unique_id.fromRandom())
        resource.rapp = self.resource_type
        resource.uri = uri
        cmd_vel_remapped, compressed_image_topic_remapped, map_remapped, scan_remapped, robot_pose_remapped = self._get_remapped_topic(rocon_uri.parse(resource.uri).name.string)
        resource.remappings = [rocon_std_msgs.Remapping(self._default_cmd_vel_topic, cmd_vel_remapped), rocon_std_msgs.Remapping(self._default_compressed_image_topic, compressed_image_topic_remapped), rocon_std_msgs.Remapping(self._default_map_topic, map_remapped), rocon_std_msgs.Remapping(self._default_scan_topic, scan_remapped), rocon_std_msgs.Remapping(self._default_robot_pose_topic, robot_pose_remapped)]
        return resource

    def _get_remapped_topic(self, name):
        '''
          Sets up remapping rules for Rapp configuration
        '''
        cmd_vel_remapped = '/' + name + self._default_cmd_vel_topic
        compressed_image_topic_remapped = '/' + name + self._default_compressed_image_topic
        map_remapped = rospy.get_param('map_topic','/map')
        scan_remapped = '/' + name + self._default_scan_topic
        robot_pose_remapped = '/' + name + '/' + self._default_robot_pose_topic

        return cmd_vel_remapped, compressed_image_topic_remapped,map_remapped, scan_remapped, robot_pose_remapped

    def loginfo(self, msg):
        rospy.loginfo("MakeAMapPimp : %s"%str(msg))

    def logwarn(self, msg):
        rospy.logwarn("MakeAMapPimp : %s"%str(msg))

    def logerr(self, msg):
        rospy.logerr("MakeAMapPimp : %s"%str(msg))


##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    rospy.init_node('make_a_map_pimp')
    pimp = MakeAMapPimp()
    rospy.spin()
    if not rospy.is_shutdown():
        pimp.cancel_all_requests()
