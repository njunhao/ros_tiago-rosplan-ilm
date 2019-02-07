#include <ros/ros.h>
#include <vector>

#include "ActionInterface/RPProbActionInterface.h"

#ifndef KCL_prob_simulatedaction
#define KCL_prob_simulatedaction

/**
 * This file defines the RPMoveBase class.
 * RPMoveBase is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPProbSimulatedActionInterface: public RPProbActionInterface
	{

	private:

		double action_duration;
		double action_probability;

	public:

		/* constructor */
		RPProbSimulatedActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
