#include <ros/ros.h>
// #include <vector>
// #include <iostream>
// #include <fstream>

#include "ActionInterface/RPProbActionInterface.h"
#include "actionlib/client/simple_action_client.h"
#include "mongodb_store/message_store.h"
#include <std_srvs/Trigger.h>

#include "geometry_msgs/PoseStamped.h"
#include "tiago_pick_demo/PickUpPoseAction.h"

#ifndef tiago_grasp
#define tiago_grasp

/**
 * This file defines the RPGrasp class.
 * RPGrasp is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPGrasp: public RPProbActionInterface
	{

	protected:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<tiago_pick_demo::PickUpPoseAction> action_client;
		ros::ServiceClient grasp_client;

		virtual void set_prob_effect();

	public:

		/* constructor */
		RPGrasp(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		// set to virtual so that function uses objects in derived class (such as action_succeeded)
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
