/**
 * Author: Chris Card
 * 
 * This class will publish controll commands to the kobuki base
 */

 #include <ros/ros.h>
// #include <yocs_controllers/default_controller.hpp>
 #include <kobuki_msgs/KeyboardInput.h>
 #include <ncurses.h>

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "keyop_controll_publisher");

 	ros::NodeHandle n;

 	ros::Publisher keyop_pub = n.advertise<kobuki_msgs::KeyboardInput>("keyop/teleop", 1000);

 	ros::Rate loop_rate(0.5);

 	int count = 0;
 	initscr(); //start cures mode
 		printw("use up arrow to increase forward velocity\n");
 		printw("use down arrow to increase reverse velocity\n");
 		printw("use left arrow to increase left motor velocity\n");
 		printw("use right arrow to increase right motor velocity\n");
 		printw("use d to disable motor\n");
 		printw("use e to enable motors\n");
 		printw("Use q to quite\n");
 		refresh();//print to real screen
 	while(ros::ok()){

 		char key = getch();
 		if(key == 'q'){ break; }
 		printw("key pressed "+key);
 		printw("\n");
 		refresh();
 		kobuki_msgs::KeyboardInput msgptr;
 		//msgptr.reset(new kobuki_msgs::KeyboardInput());

 		if (count%2 == 0){
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Right;
 		} else {
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Left;
 		}

 		ROS_INFO("%s\n", "i am turining im turingin :)");//,std::string(msgptr.pressedKey));
		refresh();
 		keyop_pub.publish(msgptr);

 		loop_rate.sleep();
 		++count;
 	}
 	endwin();
 	return 0;
 }