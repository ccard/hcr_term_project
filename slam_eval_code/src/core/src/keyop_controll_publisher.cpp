/**
 * Author: Chris Card
 * 
 * This class will publish controll commands to the kobuki base
 */

 #include <ros/ros.h>
// #include <yocs_controllers/default_controller.hpp>
 #include <kobuki_msgs/KeyboardInput.h>

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "keyop_controll_publisher");

 	ros::NodeHandle n;

 	ros::Publisher keyop_pub = n.advertise<kobuki_msgs::KeyboardInput>("keyop/teleop", 1000);

 	ros::Rate loop_rate(10);

 	int count = 0;
 	while(ros::ok()){

 		kobuki_msgs::KeyboardInputptr msgptr;
 		msgptr.reset(new kobuki_msgs::KeyboardInput());

 		if (count%2 == 0)
 		{
 			msgptr.value = kobuki_msgs::KeyboardInput::KeyCode_Right;
 		} else {
 			msgptr->value = kobuki_msgs::KeyboardInput::KeyCode_Left;
 		}

 		ROS_INFO("%s", "i am turining im turingin :)");

 		keyop_pub.publish(msgptr);

 		loop_rate.sleep();
 		++count;
 	}
 	return 0;
 }