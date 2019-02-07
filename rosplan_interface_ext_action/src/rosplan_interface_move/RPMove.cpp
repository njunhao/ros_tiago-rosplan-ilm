#include "rosplan_interface_move/RPMove.h"

/* The implementation of RPMove.h */
namespace KCL_rosplan {

	/* constructor */
	RPMove::RPMove(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), action_client(actionserver, true) {

		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	}

	/* action dispatch callback */
	bool RPMove::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("to") or 0==msg->parameters[i].key.compare("w1")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
			return false;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (%s) aborting action dispatch; no matching wpID %s", params.name.c_str(), wpID.c_str());
				prob_effect_id = 1;
				return false;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (%s) multiple waypoints share the same wpID", params.name.c_str());

			if (simulated) {
				ROS_INFO("RP+: (%s) using simulated action", params.name.c_str());
				prob_effect_id = pick_probabilistic_effect();
				if (op.probabilistic_effects.size() > 0)
					ROS_INFO("RP+: (%s) %d-th probabilistic effect chosen", params.name.c_str(), prob_effect_id);
				ROS_INFO("RP+: (%s) action finished: SUCCEEDED", params.name.c_str());
				return true;
			}

			ROS_INFO("KCL: (%s) waiting for move_base action server to start", params.name.c_str());
			action_client.waitForServer();

			// dispatch MoveBase action
			move_base_msgs::MoveBaseGoal goal;
			geometry_msgs::PoseStamped &pose = *results[0];
			goal.target_pose = pose;
			action_client.sendGoal(goal);

			bool finished_before_timeout = action_client.waitForResult();
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

				if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
					prob_effect_id = 0;
					// publish feedback (achieved)
					return true;

				} else {

					// clear costmaps
					std_srvs::Empty emptySrv;
					clear_costmaps_client.call(emptySrv);
					prob_effect_id = 1;
					// publish feedback (failed)
					return false;
				}
			} else {
				// timed out (failed)
				action_client.cancelAllGoals();
				ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
				prob_effect_id = 1;
				return false;
			}
		} else {
			// no KMS connection (failed)
			ROS_INFO("KCL: (%s) aborting action dispatch; query to sceneDB failed", params.name.c_str());
			prob_effect_id = 1;
			return false;
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_move");
		ros::NodeHandle nh("~");

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/move_base"));

		// create PDDL action subscriber
		KCL_rosplan::RPMove rpmv(nh, actionserver);

		rpmv.runActionInterface();

		return 0;
	}
