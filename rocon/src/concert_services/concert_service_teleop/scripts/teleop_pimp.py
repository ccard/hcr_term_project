#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out teleop operations for rocon interactions.
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


class TeleopPimp(concert_service_utilities.ResourcePimp):

    _default_cmd_vel_topic = '/teleop/cmd_vel'
    _default_compressed_image_topic = '/teleop/compressed_image'

    def setup_variables(self): 
        '''
            Need to setup the following variables
            service_priority, service_id, resource_type, available_resource_publisher_name, capture_topic_name
        '''
        (service_name, service_description, service_priority, service_id) = concert_service_utilities.get_service_info()
        self.service_priority = service_priority
        self.service_id = service_id
        self.resource_type = 'rocon_apps/video_teleop'
        self.available_resource_publisher_name = 'available_teleops'
        self.capture_topic_name = 'capture_teleop'

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
                self.logwarn("couldn't capture teleopable robot [not available][%s]" % msg.rocon_uri)
                response.result = False
            else:
                resource = self._create_resource(msg.rocon_uri)
                request_result, resource_request_id = self.send_allocation_request(resource)
                response.result = request_result
                if request_result == False:
                    self.logwarn("couldn't capture teleopable robot [timed out][%s]" % msg.rocon_uri)
                else:
                    self.loginfo("captured teleopable robot [%s][%s]" % (msg.rocon_uri, resource_request_id))
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
        cmd_vel_remapped, compressed_image_topic_remapped = self._get_remapped_topic(rocon_uri.parse(resource.uri).name.string)
        resource.remappings = [rocon_std_msgs.Remapping(self._default_cmd_vel_topic, cmd_vel_remapped), rocon_std_msgs.Remapping(self._default_compressed_image_topic, compressed_image_topic_remapped)]
        return resource

    def _get_remapped_topic(self, name):
        '''
          Sets up remapping rules for Rapp configuration
        '''
        cmd_vel_remapped = '/' + name + self._default_cmd_vel_topic
        compressed_image_topic_remapped = '/' + name + self._default_compressed_image_topic

        return cmd_vel_remapped, compressed_image_topic_remapped 

    def loginfo(self, msg):
        rospy.loginfo("TeleopPimp : %s"%str(msg))

    def logwarn(self, msg):
        rospy.logwarn("TeleopPimp : %s"%str(msg))

    def logerr(self, msg):
        rospy.logerr("TeleopPimp : %s"%str(msg))


##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    rospy.init_node('teleop_pimp')
    pimp = TeleopPimp()
    rospy.spin()
    if not rospy.is_shutdown():
        pimp.cancel_all_requests()
