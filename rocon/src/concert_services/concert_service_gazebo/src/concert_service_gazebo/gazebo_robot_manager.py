#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of simulated gazebo robots
# for a concert. This node can be requested to trigger a rocon_launch'ed style
# terminal which embeds a standard concert client for each gazebo robot. It
# then flips across the necessary gazebo simulated handles to that concert
# client

##############################################################################
# Imports
##############################################################################

import copy
import os
import tempfile
import yaml

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_launch
import rospy
import rocon_gateway_utils
import rocon_python_utils

from .robot_manager import RobotManager

##############################################################################
# Utility 
##############################################################################
def prepare_robot_managers(robot_type_locations, world_name):
    """
    Prepare robot manager objects based on its type
    
    :param robot_types: The configurations define robots model launcher in gazebo
    :type robot_types: [{name: robot_type, launch: <package>/<.launch>}]
    
    :returns: A dict of robot manager, and a dict of invalid robot manager
    :rtype: {robot_type: RobotManager()}, {robot_type: reason}
    """
    robot_managers = {}
    invalid_robot_managers = {}
    for name, loc in robot_type_locations.items():
        try:
            with open(loc) as f:
                loaded_robot_type = yaml.load(f)
                robot_managers[loaded_robot_type['name']] = RobotManager(loaded_robot_type, world_name)
        except rospkg.ResourceNotFound as e: 
            invalid_robot_managers[name] = str(e) 
    return robot_managers, invalid_robot_managers

##############################################################################
# Gazebo Robot Manager
##############################################################################

class GazeboRobotManager:
    '''
      This class contains all the robot-independent functionality to launch
      robots in gazebo, create concert clients for each robot, and flip
      necessary information to each concert client, so each robot can truly
      behave as a concert client.
    '''

    def __init__(self, world_name, concert_name):
        """
        :param world_name: Gazebo World name
        """
        self.is_disabled = False
        self._robot_manager = None
        self._world_name = world_name
        self._concert_name = concert_name
        self._processes = []
        self._temporary_files = []
        self._robots = []

        # Gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

        # extract spawnable robot types from package exports
        self._robot_types = self._extract_robot_types()

        # Terminal type for spawning
        try:
            self._terminal = rocon_launch.create_terminal()
        except (rocon_launch.UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            self.logwarn('cannot find a suitable terminal, falling back to spawning inside the current one [%s]' % str(e))
            self._terminal = rocon_launch.create_terminal(rocon_launch.terminals.active)

    def _extract_robot_types(self):
        cached_robot_type_information, unused_invalid_robot_types = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_GAZEBO_ROBOT_TYPE)

        self.loginfo(str(cached_robot_type_information))
        self._cached_robot_type_locations = {cached_resource_name: cached_filename for cached_resource_name, (cached_filename, unused_catkin_package) in cached_robot_type_information.iteritems()}
        self._robot_managers, self._invalid_robot_managers = prepare_robot_managers(self._cached_robot_type_locations, self._world_name)
        self.loginfo(str(self._robot_managers))
        self.loginfo(str(self._invalid_robot_managers))

        for name, manager in self._robot_managers.items():
            self.loginfo('%s loaded'%name)


    def _spawn_simulated_robots(self, robots, robot_managers):
        """
        Names and locations of robots to be spawned, read from a parameter file.

        :param robots list of dicts[]: The parameter file.
        The parameter file should read into a list of dictionaries, where each
        dict contains a "name" string, and a "location" tuple. For example:
            [{'name': 'kobuki', 'location': [0.0, 0.0, 0.0]},
             {'name': 'guimul', 'location': [0.0, 2.0, 0.0]}]
        For a full definition of the location vector, see
        RobotManager.spawn_robot().
        """
        for robot in robots:
            try:
                args = robot['args'] if 'args' in robot else None
                robot_managers[robot['type']].spawn_robot(robot["name"], robot["location"], args)
                self._robots.append(robot["name"])
            # TODO add failure exception
            except rospy.ROSInterruptException:
                self.loginfo("shutdown while spawning robot")
                continue

    def _launch_robot_clients(self, robots):
        """
        Spawn concert clients for given named robot.

        :param robot_names str[]: Names of all robots.
        """
        # spawn the concert clients
        rocon_launch_text = self._prepare_rocon_launch_text(robots)
        self.loginfo("constructing robot client rocon launcher")
        #print("\n" + console.green + rocon_launch_text + console.reset)
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        temp.write(rocon_launch_text)
        temp.close()
        launch_configurations = rocon_launch.parse_rocon_launcher(temp.name, "--screen")
        try:
            os.unlink(temp.name)
        except OSError:
            self.logerr("failed to unlink the rocon launcher.")

        for launch_configuration in launch_configurations:
            self.loginfo("launching concert client %s on port %s" %
                      (launch_configuration.name, launch_configuration.port))
            #print("%s" % launch_configuration)
            process, meta_roslauncher = self._terminal.spawn_roslaunch_window(launch_configuration)
            self._processes.append(process)
            self._temporary_files.append(meta_roslauncher)

    def _prepare_rocon_launch_text(self, robots):
        port = 11411
        launch_text  = '<concert>\n'
        for robot in robots:
            launch_text += '  <launch title="%s:%s" package="concert_service_gazebo" name="robot.launch" port="%s">\n'%(robot['name'], str(port), str(port))
            launch_text += '    <arg name="robot_name" value="%s"/>\n' % robot['name']
            launch_text += '    <arg name="robot_type" value="%s"/>\n' % robot['type'] 
            launch_text += '    <arg name="robot_concert_whitelist" value="%s"/>\n' % self._concert_name 
            launch_text += '    <arg name="robot_rapp_whitelist" value="%s"/>\n' % str(robot['robot_rapp_whitelist'])
            launch_text += '  </launch>'
            port = port + 1
        launch_text += '</concert>\n'

        return launch_text

    def _establish_unique_names(self, robots):
        """
        Make sure robot names don't clash with currently spawned robots, or
        with other robots in the same list itself. If they do, postfix them
        with an incrementing counter.

        :param robots list of dicts[]: The parameter file defining robots and
            start locations. For a full description, see
            _spawn_simulated_robots().
        :return [dict]: uniquified names for the concert clients.
        """
        unique_robots = []
        unique_robot_names = []
        for robot in robots:
            robot_name = robot["name"]
            name_extension = ''
            count = 0
            while (robot_name + name_extension in unique_robot_names or
                   robot_name + name_extension in self._robots):
                name_extension = str(count)
                count = count + 1
            unique_robot_names.append(robot_name + name_extension)
            robot_copy = copy.deepcopy(robot)
            robot_copy["name"] = robot_name + name_extension
            unique_robots.append(robot_copy)
        return unique_robots

    def _send_flip_rules(self, robots, cancel):
        """
        Flip rules from Gazebo to the robot's concert client.

        :param robot_names str[]: Names of robots to whom information needs to
            be flipped.
        :param cancel bool: Cancel existing flips. Used during shutdown.
        """
        for robot in robots:
            rules = self._robot_managers[robot['type']].get_flip_rule_list(robot['name'])
            # send the request
            request = gateway_srvs.RemoteRequest()
            request.cancel = cancel
            remote_rule = gateway_msgs.RemoteRule()
            remote_rule.gateway = robot['name'] 
            for rule in rules:
                remote_rule.rule = rule
                request.remotes.append(copy.deepcopy(remote_rule))
            try:
                self._gateway_flip_service(request)
            except rospy.ServiceException:  # communication failed
                self.logerr('failed to send flip rules')
                return
            except rospy.ROSInterruptException:
                self.loginfo('shutdown while contacting the gateway flip service')
                return

    def spawn_robots(self, robots):
        """
        Ensure all robots have existing names, spawn robots in gazebo, launch
        concert clients, and flip necessary information from gazebo to each
        concert client.

        :param robots list of dicts[]: The parameter file defining robots, type and
            start locations. For a full description, see
            _spawn_simulated_robots().
        :type robots: [{name: str, type: str, location: (x,y,theta)}]
        """
        unique_robots = self._establish_unique_names(robots)
        self._spawn_simulated_robots(unique_robots, self._robot_managers)
        self._launch_robot_clients(unique_robots)
        self._send_flip_rules(unique_robots, cancel=False)
        self._robots = unique_robots

    def spin(self):
        try:
            while not rospy.is_shutdown() and not self.is_disabled:
                rospy.sleep(0.3)
        except rospy.ROSInterruptException:
            pass
        self.shutdown()

    def shutdown(self):
        """
          - Send unflip requests.
          - Cleanup robots in gazebo.
          - Shutdown spawned terminals.
        """
        for robot in self._robots:
            try:
                self._robot_managers[robot['type']].delete_robot(robot['name'])
                #TODO quitely fail exception here
            except rospy.ROSInterruptException:
                break  # quietly fail

        self._terminal.shutdown_roslaunch_windows(processes=self._processes,
                                                  hold=False)
        for temporary_file in self._temporary_files:
            #print("Unlinking %s" % temporary_file.name)
            try:
                os.unlink(temporary_file.name)
            except OSError as e:
                self.logerr('failed to unlink temporary file [%s]' % str(e))

    def loginfo(self, msg):
        rospy.loginfo('GazeboRobotManager : %s'%str(msg))

    def logerr(self, msg):
        rospy.logerr('GazeboRobotManager : %s'%str(msg))

    def logwarn(self, msg):
        rospy.logwarn('GazeboRobotManager : %s'%str(msg))
