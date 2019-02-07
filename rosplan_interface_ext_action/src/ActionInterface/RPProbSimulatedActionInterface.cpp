#include "ActionInterface/RPProbSimulatedActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPProbSimulatedActionInterface::RPProbSimulatedActionInterface(ros::NodeHandle &nh) {
		action_duration = 0.0;
		action_probability = 1.0;
		nh.getParam("action_duration", action_duration);
		nh.getParam("action_probability", action_probability);
	}

	/* action dispatch callback */
	// todo: return probabilistic effect used and action_success
	bool RPProbSimulatedActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// #actionDispatch message
		// int32 action_id
		// string name
		// diagnostic_msgs/KeyValue[] parameters
		// float32 duration
		// float32 dispatch_time

		ROS_INFO("RP+: (%s) Action completing with probability %f and duration %f", params.name.c_str(), action_probability, action_duration);

		// wait for some time
		if(action_duration > 0) {
			ros::Rate wait = 1.0 / action_duration;
			wait.sleep();
		}

		// complete the action
		if ((rand() % 100) <= (100 * action_probability)) {
			// action succeed, now choose probabilistic effect
			float probability = (rand() % 100) / 100.0;
			float cumsum_probability = 0.0;
			prob_effect_id = pick_probabilistic_effect();
			if (op.probabilistic_effects.size() > 0) 
				ROS_INFO("RP+: (%s) %d-th probabilistic effect chosen", params.name.c_str(), prob_effect_id);
			return true;
		}
		
		return false;   // action fail
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_simulated_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPProbSimulatedActionInterface rpsa(nh);

		rpsa.runActionInterface();

		return 0;
	}
