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
 	cbreak();
 	noecho();
 	keypad(stdscr, true);
 	intrflush(stdscr,false);
 		
 	while(ros::ok()){

 		if(count%10 == 0){
 			clear();
 			printw("use up arrow to increase forward velocity\n");
 			printw("use down arrow to increase reverse velocity\n");
 			printw("use left arrow to increase left motor velocity\n");
	 		printw("use right arrow to increase right motor velocity\n");
 			printw("use d to disable motor\n");
 			printw("use e to enable motors\n");
 			printw("Use q to quite\n");
		}
		//refresh();//prints to real screen
 		
 		char key = getch();
 		if(key == 'q'){ break; }
 		printw("%d\n",key);
 		
 		kobuki_msgs::KeyboardInput msgptr;

 		switch(key){
 		case 2: //keydown
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Down;
 			keyop_pub.publish(msgptr);
 			break;
 		case 3://KEY_UP:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Up;
 			keyop_pub.publish(msgptr);
 			break;
 		case 5://KEY_RIGHT:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Right;
 			keyop_pub.publish(msgptr);
 			break;
 		case 4://KEY_LEFT:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Left;
 			keyop_pub.publish(msgptr);
 			break;
 		case 'e':
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Enable;
 			keyop_pub.publish(msgptr);
 			break;
 		case 'd':
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Disable;
 			keyop_pub.publish(msgptr);
 			break;
 		case kobuki_msgs::KeyboardInput::KeyCode_Space:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Space;
 			keyop_pub.publish(msgptr);
 			break;
 		default:
 			//TODO: publish to other threads for the slame methods
 			break;
 		}

 		//ROS_INFO("%s\n", "i am turining im turingin :)");//,std::string(msgptr.pressedKey));

 		printw("%s\n","I am now publishing!");
		refresh();

 		loop_rate.sleep();
 		++count;

 	}
 	endwin();
 	return 0;
 }