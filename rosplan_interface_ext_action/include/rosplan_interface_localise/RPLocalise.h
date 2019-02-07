#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

// #include "actionlib/client/simple_action_client.h"
#include "ActionInterface/RPProbActionInterface.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "rosplan_dispatch_msgs/ParsingService.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/SetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <std_msgs/Float64.h>

#ifndef ext_localise
#define ext_localise

#define PI 3.14159265358979323846  /* pi */

namespace KCL_rosplan {

	class RPLocalise: public RPProbActionInterface
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		ros::ServiceClient clear_costmaps_client, global_localization_client, set_map_client;
		ros::Publisher cmd_vel_pub;
		ros::Subscriber pose_sub, ground_truth_pose_sub, map_sub;
		geometry_msgs::PoseWithCovariance pose, true_pose;
		nav_msgs::OccupancyGrid cost_map;

		// void poseCallback(const nav_msgs::Odometry& msg);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
		void truePoseCallback(const nav_msgs::Odometry& msg);
		void costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
		bool accurate_localisation() const;

	public:

		/* constructor */
		RPLocalise(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};

	double convert2Euler(const geometry_msgs::Pose& pose);
}
#endif
