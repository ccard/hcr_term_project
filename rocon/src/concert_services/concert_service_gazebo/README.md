# Using the Concert Framework with Gazebo (the short short version)

There is some code that is robot-agnostic. All that code is in this package in the [GazeboRobotManager Module](src/concert_service_gazebo/gazebo_robot_manager.py)

## GazeboRobotManager

* Creates concert clients for each robot
* Flips necessary information to that robot's concert client, so that it can pretend to be a real robot.

Unfortunately, a lot of code depends on the robot that you are going to simulate. For each robot, you'll have to implement a module specific to that robot. An abstract module detailing this API is [RobotManager](src/concert_service_gazebo/robot_manager.py)  

## RobotManager

* Specifies how to spawn the robot in Gazebo
* Specifies how to delete the robot in Gazebo
* Specifies which information needs to be flipped over. If you're doing teleop, cmd_vel is sufficient. If you want to do full navigation, you'll have to supply tf,map,sensory information and odometry as well.
* Specifies how each concert client for these simulated robots should be created.

# Some other things to consider

* The concert solution and all subsequent concert clients need to be started with /use_sim_time set to true.
* Since all robots are being simulated on a single master, the tf tree needs tobe properly namespaced for each robot. Furthermore, applications run by this spawned concert clients need to use this namespaced tf tree. 


# Robot configuration Parameter

[Example parameter](https://github.com/robotics-in-concert/concert_services/blob/gazebo_upgrade/concert_service_gazebo/services/gazebo_robot_world/gazebo_robot_world.parameters)

```
robots:
  - name: guimul
    type: kobuki
    robot_rapp_whitelist: [rocon_apps, concert_service_gazebo]
    location: [0.0, 0.0, 0.0]
  - name: gamza 
    type: turtlebot
    robot_rapp_whitelist: [rocon_apps, concert_service_gazebo]
    location: [0.0, -2.0, 3.14159265359]
  - name: doldol
    type: segbot
    robot_rapp_whitelist: [rocon_apps, concert_service_gazebo]
    location: [0.0, 4.0, 0.0]
types:
  - name: kobuki 
    launch: concert_service_gazebo/kobuki.launch
    flip_rule:
      pub:
        - odom
      sub:
        - mobile_base/.*
  - name: turtlebot
    launch: concert_service_gazebo/turtlebot.launch
    flip_rule:
      pub:
        - odom
        - camera/.*
      sub:
        - cmd_vel_mux/.*
  - name: segbot
    launch: concert_service_gazebo/segbot.launch 
    flip_rule:
      pub:
        - odom
        - scan_filtered
      sub:
        - cmd_vel
world_file: concert_service_gazebo/empty_world.world
```


# An example

Take a look at the [gazebo_solution](https://github.com/robotics-in-concert/rocon_demos/tree/gazebo_concert/gazebo_solution) demo.

* Install Rocon:

> wstool init . https://raw.githubusercontent.com/robotics-in-concert/rocon_demos/gazebo_concert/gazebo_solution.rosinstall

* Run the demo:
```
> roslaunch gazebo_solution concert.launch --screen
> rocon_remocon # Teleop..
```

```
> roslaunch office_sim_solution concert.launch --screen
> rocon_remocon # Teleop..
```
