#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: subscriber_proxy
   :platform: Unix
   :synopsis: Request-response style communication with a latched publisher.


This module provides a means of interacting with a ros latched publisher
in the same style as you would a ros service (request-response).

----

"""
##############################################################################
# Imports
##############################################################################

import time
import rospy

##############################################################################
# Subscriber Proxy
##############################################################################


class SubscriberProxy():
    '''
    Works like a service proxy, but using a latched subscriber instead (regular
    subscribers will also work, but this is especially useful for latched
    subscribers since they typically always provide data).

    If no timeout is specified when calling, it blocks indefinitely on a
    100ms loop until a message arrives. Alternatively it will return with None
    if a specified timeout is reached.

    **Usage:**

    .. code-block:: python

        from rocon_python_comms import SubscriberProxy

        gateway_info = SubscriberProxy('gateway_info', gateway_msgs.GatewayInfo)(rospy.Duration(0.5))
        if gateway_info is not None:
            # do something

    :todo: upgrade to make use of python events instead of manual loops
    '''
    def __init__(self, topic, msg_type):
        '''
        :param str topic: the topic name to subscriber to
        :param str msg_type: any ros message type (e.g. std_msgs/String)
        :param rospy.Duration timeout: timeout on the wait operation (None = /infty)
        :returns: msg type data or None
        :rtype: same as the msg type specified in the arg or None
        '''
        self._subscriber = rospy.Subscriber(topic, msg_type, self._callback)
        self._data = None

    def __call__(self, timeout=None):
        '''
          Returns immediately with the latest data or blocks indefinitely until
          the next data arrives.

          :param rospy.Duration timeout: time to wait for data, polling at 10Hz.
          :returns: latest data or None
        '''
        if timeout:
            # everything in floating point calculations
            timeout_time = time.time() + timeout.to_sec()
        while not rospy.is_shutdown() and self._data == None:
            rospy.rostime.wallsleep(0.1)
            if timeout:
                if time.time() > timeout_time:
                    return None
        return self._data

    def wait_for_next(self, timeout=None):
        '''
          Makes sure any current data is cleared and waits for new data.

          :param rospy.Duration timeout: time to wait for data, polling at 10Hz.
          :returns: latest data or None
        '''
        self._data = None
        return self.__call__(timeout)

    def wait_for_publishers(self):
        '''
          Blocks until publishers are seen.

          :raises: rospy.ROSInterruptException if we are in shutdown.
        '''
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._subscriber.get_num_connections() != 0:
                return
            else:
                r.sleep()
        # we are shutting down
        raise rospy.exceptions.ROSInterruptException

    def _callback(self, data):
        self._data = data

    def unregister(self):
        '''
          Unregister the subscriber so future instantiations of this class can pull a
          fresh subscriber (important if the data is latched).
        '''
        self._subscriber.unregister()
