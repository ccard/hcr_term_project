/**
* Author: Chris Card, Marshall Sweat
* 
* This program is ment to work on the master computer and publish commands
* to the turtlebot to controll mostion and slam
*/

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <kobuki_msgs/KeyboardInput.h>

namspace kobuki
{
	class KeyboardController: public yocs::Controller
	{
	public:
		KeyboardController(ros::NodeHandle &nh, std::string &name): Controller(), nh_(nh),name_(name){};
		~KeyboardController(){};
		
		bool init(){
			enable_controller_subscriber_ = nh_.subscribe("enable",10, &KeyboardController::enableKC, this);
			disable_controller_subscriber_ = nh_.subscribe("disable",10, &KeyboardController::disableKC, this);
			key_publisher_ = nh_.advertise<kobuki_msgs::KeyboardInput>("keyop/teleop",10);
			return true;
		};

	private:
		ros::NodeHandle nh_;
		std::string name_;
		ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
		ros::Publisher key_publisher_;

		/**
		 * logging ouput for enabling controller
		 * @param: msg incoming topic message
		 */
		void enableKC(const std_msgs::EmptyConstPtr msg);


		/**
		 * logging disabling controller
		 * @param: msg incoming topic msg
		 */
		void disableKC(const std_msgs::EmptyConstPtr msg);
	};

	void KeyboardController::enableKC(const std_msgs::EmptyConstPtr msg){
		if(this->enable()){
			ROS_INFO_STREAM("Controller has been enabled.[" << name_ << "]");
		} else {
			ROS_INFO_STREAM("Controller is already enabled.[" << name_ << "]");
		}
	};

	void KeyboardController::disableKC(const std_msgs::EmptyConstPtr msg){
		if(this->disable()){
			ROS_INFO_STREAM("Controller has been disabled.[" << name_ << "]");
		} else {
			ROS_INFO_STREAM("Controller was already disabled.[" << name_ << "]");
		}
	};
}