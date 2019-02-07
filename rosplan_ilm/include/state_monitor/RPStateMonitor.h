#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "kb_agent/KBAgent.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "rosplan_ilm_msgs/GetInstanceCount.h"
#include "rosplan_ilm_msgs/SetReservedInstance.h"


#ifndef ilm_state_monitor
#define ilm_state_monitor


namespace KCL_rosplan {

	class RPStateMonitor: public RPKBAgent
	{

	private:

		std::string robot_instance;
		std::vector<std::string> reserved_instances, tmp_reserved_instances;
		ros::Subscriber pose_sub;
		ros::ServiceServer GetInstanceCount, setReservedInstance;
		bool check_localisation;

		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
		bool accurate_localisation(const geometry_msgs::PoseWithCovarianceStamped& pose) const;
		bool remove_unused_instance(rosplan_ilm_msgs::GetInstanceCount::Request &req, rosplan_ilm_msgs::GetInstanceCount::Response &res);
		bool set_reserved_instance(rosplan_ilm_msgs::SetReservedInstance::Request &req, rosplan_ilm_msgs::SetReservedInstance::Response &res);
		bool is_reserved_instance(const std::string& instance) const;


	public:

		/* constructor */
		RPStateMonitor(ros::NodeHandle &nh);

	};

}
#endif
