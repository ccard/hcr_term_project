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

"""Provides functions to produce dotcode which represents the capability graph
"""

from qt_dotgraph.pydotfactory import PydotFactory


def generate_dotcode_from_capability_info(spec_index, running_providers):
    dotcode_factory = PydotFactory()
    dotgraph = dotcode_factory.get_graph(rankdir="BT")
    interface_graphs = {}
    # Draw plain interfaces
    for name in spec_index.interfaces:
        providers = [k for k, v in spec_index.providers.items() if v.implements == name]
        # Only create a subgraph if it has providers
        if providers:
            interface_graphs[name] = dotcode_factory.add_subgraph_to_graph(dotgraph, str(name) + "_group", subgraphlabel='')
        # Draw box for interface
        graph = interface_graphs.get(name, dotgraph)
        dotcode_factory.add_node_to_graph(graph, nodename=str(name), shape="box")
    # Draw semantic interfaces
    for name, interface in spec_index.semantic_interfaces.items():
        providers = [k for k, v in spec_index.providers.items() if v.implements == name]
        # Only create a subgraph if it has providers
        if providers:
            interface_graphs[name] = dotcode_factory.add_subgraph_to_graph(dotgraph, str(name) + "_group", subgraphlabel='')
        graph = interface_graphs.get(name, dotgraph)
        # Draw box for semantic interface
        dotcode_factory.add_node_to_graph(graph, nodename=str(name), shape="box")
        # Make edge to interface it redefines, if it exists
        if interface.redefines in spec_index.interfaces:
            dotcode_factory.add_edge_to_graph(dotgraph, str(name), str(interface.redefines), label="redefines")
    # Draw providers
    interfaces = dict(spec_index.interfaces)
    interfaces.update(spec_index.semantic_interfaces)
    for name, provider in spec_index.providers.items():
        # Get subgraph of interface this provider implements
        graph = interface_graphs[provider.implements]
        # Get the default provider for the interface this provider implements
        default_provider = interfaces[provider.implements].default_provider
        provider_name = name
        # Add annotaion if this is the default provider
        if default_provider != 'unknown' and default_provider == name:
            provider_name += "  (default)"
        # If it is running, make it green
        if name in running_providers:
            dotcode_factory.add_node_to_graph(graph, nodename=str(name), nodelabel=str(provider_name), shape="ellipse", color="green")
        # Else no color
        else:
            dotcode_factory.add_node_to_graph(graph, nodename=str(name), nodelabel=str(provider_name), shape="ellipse")
        # Add edges to the interface, provider paris this provider depends on
        for dep, relationship in provider.dependencies.items():
            if relationship.preferred_provider is not None:
                dotcode_factory.add_edge_to_graph(dotgraph, str(name), str(relationship.preferred_provider), label="requires")
            elif spec_index.interfaces[relationship.capability_name].default_provider != 'unknown':
                dotcode_factory.add_edge_to_graph(dotgraph, str(name), str(spec_index.interfaces[relationship.capability_name].default_provider), label="requires")
    return dotcode_factory.create_dot(dotgraph)
