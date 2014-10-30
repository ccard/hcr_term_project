#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: services
   :platform: Unix
   :synopsis: Useful methods relating to ros services.


This module contains anything relating to introspection or manipulation
of ros services.

----

"""
##############################################################################
# Imports
##############################################################################

import rosgraph
import rospy
import socket
import time
from rosservice import ROSServiceIOException, get_service_headers

# Local imports
from .exceptions import NotFoundException

##############################################################################
# Find topics/services
##############################################################################


def find_service(service_type, timeout=rospy.rostime.Duration(5.0), unique=False):
    '''
    Do a lookup to find services of the type
    specified. This will raise exceptions if it times out or returns
    multiple values. It can apply the additional logic of whether this should
    return a single unique result, or a list. Under the hood this calls out to the ros master for a list
    of registered services and it parses that to determine the result. If nothing
    is found, it loops around internally on a 10Hz loop until the result is
    found or the specified timeout is reached.

    Usage:

    .. code-block:: python

        from rocon_python_comms import find_service

        try:
            service_name = rocon_python_comms.find_service('rocon_interaction_msgs/SetInteractions',
                                                           timeout=rospy.rostime.Duration(15.0),
                                                           unique=True)
        except rocon_python_comms.NotFoundException as e:
            rospy.logwarn("failed to find the set_interactions service.")

    :param str service_type: service type specification, e.g. concert_msgs/GetInteractions
    :param rospy.Duration timeout: raise an exception if nothing is found before this timeout occurs.
    :param bool unique: flag to select the lookup behaviour (single/multiple results)

    :returns: the fully resolved name of the service (unique) or list of names (non-unique)
    :rtype: str

    :raises: :exc:`.NotFoundException`
    '''
    # we could use rosservice_find here, but that throws exceptions and aborts if it comes
    # across any rosservice on the system which is no longer valid. To be robust against this
    # I've just pulled the internals of rosservice_find (ugh its fugly) and replicated it here
    # with the only difference in that I continue over that exception.
    unique_service_name = None
    service_names = []
    timeout_time = time.time() + timeout.to_sec()
    master = rosgraph.Master(rospy.get_name())
    while not rospy.is_shutdown() and time.time() < timeout_time and not service_names:
        services_information = []
        try:
            _, _, services = master.getSystemState()
            for service_name, unused_node_name in services:
                service_uri = master.lookupService(service_name)
                services_information.append((service_name, service_uri))
        except (rosgraph.masterapi.Error, rosgraph.masterapi.Failure, socket.error) as e:
            raise NotFoundException("unable to communicate with the master [%s]" % str(e))
        for (service_name, service_uri) in services_information:
            try:
                next_service_type = get_service_headers(service_name, service_uri).get('type', None)
            except ROSServiceIOException:  # should also catch socket.error?
                # ignore this - it is usually a sign of a bad service that could be thrown
                # up by somebody else and not what we're trying to find. If we can skip past it
                # here we can be robust to other people's problems.
                continue
            if next_service_type == service_type:
                service_names.append(service_name)
        if unique:
            if len(service_names) > 1:
                raise NotFoundException("multiple services found %s." % service_names)
            elif len(service_names) == 1:
                unique_service_name = service_names[0]
        if not service_names:
            rospy.rostime.wallsleep(0.1)
    if not service_names:
        raise NotFoundException("timed out")

    return unique_service_name if unique_service_name else service_names
