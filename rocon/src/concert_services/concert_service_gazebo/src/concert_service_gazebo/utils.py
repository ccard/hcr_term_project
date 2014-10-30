#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################

import os
import subprocess
import tempfile

# Utilities 
def reformat_position_vector(position_vector):
    """
    Reformats position vector so that it can be used with the segbot launch script
    """
    if len(position_vector) == 2:
        reformatted_vector = [position_vector[0], position_vector[1], 0.0, 0.0, 0.0, 0.0]
    elif len(position_vector) == 3:
        reformatted_vector = [position_vector[0], position_vector[1], 0.0, 0.0, 0.0, position_vector[2]]
    else:
        reformatted_vector = [position_vector[x] if x < len(position_vector) else 0.0 for x in range(6)]

    location = {}
    location['loc_x']     = reformatted_vector[0]
    location['loc_y']     = reformatted_vector[1]
    location['loc_z']     = reformatted_vector[2]
    location['loc_roll']  = reformatted_vector[3]
    location['loc_pitch'] = reformatted_vector[4]
    location['loc_yaw']   = reformatted_vector[5]

    return location


def generate_spawn_robot_launch_script(name, location, world_namespace, launch_file, args):
    """
    Generates the roslaunch script for a single robot, wrapping the appropriate
    launch file in gazebo robot 
    """
    launch_text  = '<launch>\n' 
    launch_text += '  <include ns="%s" file="%s">\n' %(name, launch_file)
    launch_text += '    <arg name="name" value="%s"/>\n'%(name)
    launch_text += '    <arg name="world_namespace" value="%s"/>\n'%world_namespace
    for loc_key, loc_value in location.items():
        launch_text += '    <arg name="%s" value="%s"/>\n'%(loc_key, loc_value)

    if args:
        for arg_name, arg_value in args.items():
            launch_text += '    <arg name="%s" value="%s"/>\n'%(arg_name, arg_value)
    launch_text += '  </include>\n'
    launch_text += '</launch>'
    return launch_text


def start_roslaunch_process(launch_script):
    """
    Robots are spawned using roslaunch instead of gazebo/spawn_model so that
    a few other scripts such as robot_state_publisher can also be launched.
    This convenience function helps launch a roslaunch script
    """
    temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
    temp.write(launch_script)
    temp.close()
    command_args = ['roslaunch', temp.name]
    command_args.append('--screen')
    roslaunch_env = os.environ.copy()
    try:
        # ROS_NAMESPACE gets set since we are inside a node here
        # got to get rid of this otherwise it pushes things down
        del roslaunch_env['ROS_NAMESPACE']
    except KeyError:
        pass
    process = subprocess.Popen(command_args, env=roslaunch_env)
    return process
