# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: William Woodall <william@osrfoundation.org>

"""Extends the InteractiveGraphicsView from rqt_graph to handle special actions
"""

import rospy

from PySide.QtCore import Qt
from PySide.QtCore import QPoint

from PySide.QtGui import QMenu

from rqt_graph.interactive_graphics_view import InteractiveGraphicsView

from qt_dotgraph.node_item import NodeItem

from capabilities.srv import StartCapability
from capabilities.srv import StopCapability


class CapabilitiesInteractiveGraphicsView(InteractiveGraphicsView):
    """Extends the InteractiveGraphicsView from rqt_graph"""
    def __init__(self, parent=None):
        super(InteractiveGraphicsView, self).__init__(parent)
        self._last_pan_point = None
        self._last_scene_center = None
        self._running_providers = []
        self._spec_index = None

    def mousePressEvent(self, mouse_event):
        if mouse_event.button() == Qt.RightButton:
            if self._spec_index is None:
                print("spec_index is None")
                print(self)
                print(self._running_providers)
                return
            pos = mouse_event.pos()
            items = [item for item in self.items(pos) if isinstance(item, NodeItem) and item._label.text()]
            if len(items) != 1:
                print("wrong number of things", [x._label.text() for x in items])
                return

            name = items[0]._label.text().rstrip('(default)').strip()
            if name not in self._spec_index.provider_names:
                print(name, "Not in list of providers")
                return
            provider = self._spec_index.providers[name]

            def start_trigger():
                # TODO: replace 'capability_server' with user provided server name
                service_name = '/{0}/start_capability'.format('capability_server')
                rospy.wait_for_service(service_name)
                start_capability = rospy.ServiceProxy(service_name, StartCapability)
                start_capability(provider.implements, name)

            def stop_trigger():
                # TODO: replace 'capability_server' with user provided server name
                service_name = '/{0}/stop_capability'.format('capability_server')
                rospy.wait_for_service(service_name)
                stop_capability = rospy.ServiceProxy(service_name, StopCapability)
                stop_capability(provider.implements)

            if name not in self._running_providers:
                trigger = start_trigger
                msg = "start => "
            else:
                trigger = stop_trigger
                msg = "stop => "

            menu = QMenu()
            action = menu.addAction(msg + name)
            action.triggered.connect(trigger)
            pos = mouse_event.globalPos()
            pos = QPoint(pos.x(), pos.y())
            menu.exec_(pos)
        else:
            InteractiveGraphicsView.mousePressEvent(self, mouse_event)
