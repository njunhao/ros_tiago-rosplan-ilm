#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "ActionInterface/RPProbActionInterface.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "std_srvs/Empty.h"

#include <tf/tf.h>

#ifndef ext_explore
#define ext_explore

/**
 * This file defines the RPExplore class.
 * RPExplore is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPExplore: public RPProbActionInterface
	{

	protected:
		std::string data_path;
		nav_msgs::Path exploration_path;
		ros::Subscriber exploration_path_sub, map_sub, sub_pose;
		geometry_msgs::PoseWithCovarianceStamped robot_pose;

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
		ros::ServiceClient clear_costmaps_client, global_planner_client;

		void explorationPathCallback(const nav_msgs::Path &expath);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

		bool parse_file(nav_msgs::Path &expath);
		void greedy_path_planning(std::vector<geometry_msgs::Pose>& waypoints, const std::string frame_id);
		double euclidean_distance(geometry_msgs::Pose wp1, geometry_msgs::Pose wp2);

		virtual void start_process();
		virtual void end_process();
		virtual void set_prob_effect();
		virtual void initialize(ros::NodeHandle &nh);
		virtual bool continue_exploring();
		virtual bool is_successful();
		virtual bool run_simulated_action();
		virtual double get_path_cost(geometry_msgs::Pose wp1, geometry_msgs::Pose wp2, const std::string frame_id);


	public:

		/* constructor */
		RPExplore(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		// set to virtual so that function uses objects in derived class (such as action_succeeded)
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
