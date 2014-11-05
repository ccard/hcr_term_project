/**
* Author: Chris Card, Marshall Sweat
* 
*
*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "kobuki_teleop_controller.hpp"

namespace kobuki {

	class KeyboardControllerNodelet: public nodelet::Nodelet
	{
	public:
		KeyboardControllerNodelet(){};
		~KeyboardControllerNodelet(){};
		
		virtual void onInit(){
			ros;:NodeHandle nh = this->getPrivateNodeHandle();

			std::string name = nh.getUnresolvedNamespace();
			int pos = name.find_last_of('/');
			name = name.substr(pos+1);
			NODELET_INFO_STREAM("Initializing nodelet .. [" << name << "]");

			controller_.reset(new KeyboardController(nh,name));

			if (controller_->init())
			{
				NODELET_INFO_STREAM("Nodelet initalised [" << name << "].");
			} else {
				NODELET_INFO_STREAM("Could not initalise Nodelet [" << name << "].");
			}
		}

	private:
		boost::shared_ptr<KeyboardController> controller_;
	};
}

PLUGINLIB_EXPORT_CLASS(kobuki::KeyboardControllerNodelet,nodelet::Nodelet);