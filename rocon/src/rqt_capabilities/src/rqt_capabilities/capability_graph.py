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

"""This module implements the CapabilityGraph rqt plugin
"""

from __future__ import print_function

import os

import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtGui import QGraphicsScene
from python_qt_binding.QtGui import QWidget

from qt_dotgraph.dot_to_qt import DotToQtGenerator

from rqt_gui_py.plugin import Plugin

from rqt_capabilities.graphics_view import CapabilitiesInteractiveGraphicsView

from rqt_capabilities.dotcode import generate_dotcode_from_capability_info

from capabilities.service_discovery import spec_index_from_service

from capabilities.msg import CapabilityEvent

from capabilities.srv import GetRunningCapabilities


class CapabilityGraph(Plugin):

    __deferred_fit_in_view = Signal()
    __redraw_graph = Signal()

    def __init__(self, context):
        super(CapabilityGraph, self).__init__(context)
        self.setObjectName('CapabilityGraph')

        self.__current_dotcode = None

        self.__running_providers = []
        self.__spec_index = None

        self.__widget = QWidget()

        self.__dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_capabilities'), 'resources', 'CapabilityGraph.ui')
        loadUi(ui_file, self.__widget, {'CapabilitiesInteractiveGraphicsView': CapabilitiesInteractiveGraphicsView})
        self.__widget.setObjectName('CapabilityGraphUI')
        if context.serial_number() > 1:
            self.__widget.setWindowTitle(self.__widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.__scene = QGraphicsScene()
        self.__scene.setBackgroundBrush(Qt.white)
        self.__widget.graphics_view.setScene(self.__scene)

        self.__widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.__widget.refresh_graph_push_button.pressed.connect(self.__refresh_view)

        self.__refresh_view()
        self.__deferred_fit_in_view.connect(self.__fit_in_view, Qt.QueuedConnection)
        self.__deferred_fit_in_view.emit()
        self.__redraw_graph.connect(self.__update_capabilities_graph)

        # TODO: use user provided server node name
        rospy.Subscriber('/capability_server/events', CapabilityEvent, self.__handle_event)

        context.add_widget(self.__widget)

    def __handle_event(self, msg):
        if msg.type == CapabilityEvent.STOPPED:
            return
        if msg.type == CapabilityEvent.LAUNCHED and msg.provider not in self.__running_providers:
            self.__running_providers.append(msg.provider)
        if msg.type == CapabilityEvent.TERMINATED and msg.provider in self.__running_providers:
            self.__running_providers.remove(msg.provider)
        self.__redraw_graph.emit()

    def __get_specs(self):
        self.__spec_index, errors = spec_index_from_service()
        assert not errors

    def __get_running_providers(self):
        # TODO: replace 'capability_server' with user provided server name
        service_name = '/{0}/get_running_capabilities'.format('capability_server')
        rospy.wait_for_service(service_name)
        get_running_capabilities = rospy.ServiceProxy(service_name, GetRunningCapabilities)
        response = get_running_capabilities()
        self.__running_providers = []
        for cap in response.running_capabilities:
            self.__running_providers.append(cap.capability.provider)

    def __refresh_view(self):
        self.__get_specs()
        self.__get_running_providers()
        self.__update_capabilities_graph()

    def __update_capabilities_graph(self):
        self.__update_graph_view(self.__generate_dotcode())

    def __generate_dotcode(self):
        return generate_dotcode_from_capability_info(self.__spec_index, self.__running_providers)

    def __update_graph_view(self, dotcode):
        if dotcode == self.__current_dotcode:
            return
        self.__current_dotcode = dotcode
        self.__redraw_graph_view()

    def __fit_in_view(self):
        self.__widget.graphics_view.fitInView(self.__scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def __redraw_graph_view(self):
        self.__widget.graphics_view._running_providers = self.__running_providers
        self.__widget.graphics_view._spec_index = self.__spec_index
        self.__scene.clear()

        highlight_level = 1

        # layout graph and create qt items
        (nodes, edges) = self.__dot_to_qt.dotcode_to_qt_items(self.__current_dotcode,
                                                              highlight_level=highlight_level,
                                                              same_label_siblings=True)

        for node_item in nodes.itervalues():
            self.__scene.addItem(node_item)
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self.__scene)

        self.__scene.setSceneRect(self.__scene.itemsBoundingRect())
        self.__fit_in_view()
