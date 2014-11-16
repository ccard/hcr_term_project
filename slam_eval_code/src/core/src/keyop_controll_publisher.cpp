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
 	cbreak(); //This method means no line breaks when using getch (i.e. enter doesn't have to be hit)
 	noecho(); //Prevents getch from echoing the keyboard output
 	keypad(stdscr, true); //turns arrow keys from a 3 character sequence into numbers 2-4
 	intrflush(stdscr,false); //prevnents flushing to the standard screen (i think)
 		
 	while(ros::ok()){

 		if(count%10 == 0){
 			clear();
 			printw("up arrow:   %s", "increase forward velocity\n");
 			printw("down arrow: %s", "increase reverse velocity\n");
 			printw("left arrow: %s", "increase left motor velocity\n");
	 		printw("right arrow:%s", "increase right motor velocity\n");
 			printw("d:          %s", "disable motor\n");
 			printw("e:          %s", "enable motors\n");
 			printw("q:          %s", "quit\n");
		}
 		
 		char key = getch();
 		if(key == 'q'){ break; }
 		//printw("%d\n",key); //key checking
 		
 		kobuki_msgs::KeyboardInput msgptr;

 		switch(key){
 		case 2: //keydown
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Down;
 			keyop_pub.publish(msgptr);
 			printw("Increasing negative velocity");
 			break;
 		case 3://KEY_UP:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Up;
 			keyop_pub.publish(msgptr);
 			printw("Increasing positive velocity");
 			break;
 		case 5://KEY_RIGHT:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Right;
 			keyop_pub.publish(msgptr);
 			printw("Incrinsing right angular velocity");
 			break;
 		case 4://KEY_LEFT:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Left;
 			keyop_pub.publish(msgptr);
 			printw("Increasing left angular velocity");
 			break;
 		case 'e':
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Enable;
 			keyop_pub.publish(msgptr);
 			printw("Enabling motors");
 			break;
 		case 'd':
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Disable;
 			keyop_pub.publish(msgptr);
 			printw("Disabling motors");
 			break;
 		case kobuki_msgs::KeyboardInput::KeyCode_Space:
 			msgptr.pressedKey = kobuki_msgs::KeyboardInput::KeyCode_Space;
 			keyop_pub.publish(msgptr);
 			break;
 		default:
 			//TODO: publish to other threads for the slam methods
 			break;
 		}

 		//ROS_INFO("%s\n", "i am turining im turingin :)");//,std::string(msgptr.pressedKey));

 		//printw("%s\n","I am now publishing!");
		refresh();

 		loop_rate.sleep();
 		++count;
 	}
 	endwin();
 	return 0;
 }