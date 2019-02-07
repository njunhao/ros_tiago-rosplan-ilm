#include <ros/ros.h>
// #include <vector>
// #include <iostream>
// #include <fstream>

#include "ActionInterface/RPProbActionInterface.h"
#include "actionlib/client/simple_action_client.h"
#include "mongodb_store/message_store.h"
#include <std_srvs/Trigger.h>

#include "geometry_msgs/PoseStamped.h"
#include "play_motion_msgs/PlayMotionAction.h"


#ifndef tiago_playmotion
#define tiago_playmotion


namespace KCL_rosplan {

	class RPPlayMotion: public RPProbActionInterface
	{

	protected:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> action_client;
		std::string motion_name;

		virtual void set_prob_effect();

	public:

		/* constructor */
		RPPlayMotion(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		// set to virtual so that function uses objects in derived class (such as action_succeeded)
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
