#include "state_monitor/RPStateMonitor.h"
#include <math.h> 

#define ACCURACY 3.0
#define RAD_ACCURACY 0.174533  // 10 deg
#define LOCALISED "localised"


namespace KCL_rosplan {

	/* constructor */
	RPStateMonitor::RPStateMonitor(ros::NodeHandle &nh) {
		nh.param("robot", robot_instance, std::string("r1"));
		nh.param("check_localisation", check_localisation, false);
		if (check_localisation) {
			std::string topic;
			nh.param("pose_topic", topic, std::string("/amcl_pose"));
			pose_sub = nh.subscribe(topic, 1, &RPStateMonitor::poseCallback, this);
		}
		initialize_services();
		GetInstanceCount = nh.advertiseService("remove_unused_instance", &KCL_rosplan::RPStateMonitor::remove_unused_instance, this);
		setReservedInstance = nh.advertiseService("set_reserved_instance", &KCL_rosplan::RPStateMonitor::set_reserved_instance, this);
	}


	bool RPStateMonitor::accurate_localisation(const geometry_msgs::PoseWithCovarianceStamped& pose) const {
		// ROS_INFO("RP+: (%s) pose standard deviation is %f %f %f %f", ros::this_node::getName().c_str(), 
		// 	std::sqrt(fabs(pose.pose.covariance[0])), std::sqrt(fabs(pose.pose.covariance[7])), std::sqrt(fabs(pose.pose.covariance[14])), std::sqrt(fabs(pose.pose.covariance[35])));
		return ((std::sqrt(fabs(pose.pose.covariance[0])) < ACCURACY) 
					and (std::sqrt(fabs(pose.pose.covariance[7])) < ACCURACY) 
					and (std::sqrt(fabs(pose.pose.covariance[14])) < ACCURACY)
					and (std::sqrt(fabs(pose.pose.covariance[35])) < RAD_ACCURACY));
	}


	void RPStateMonitor::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
		if (!accurate_localisation(msg)) {
			std::string predicate_name = LOCALISED;
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> list_predicates;
			get_facts_in_kb(predicate_name, list_predicates);
			for (uint i = 0; i < list_predicates.size(); i++) {
				if (list_predicates[i].values[0].value.compare(robot_instance)==0) {
					rosplan_knowledge_msgs::KnowledgeItem item;
					item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					item.attribute_name = predicate_name;
					diagnostic_msgs::KeyValue value;
					value.key = list_predicates[i].values[0].key;
					value.value = list_predicates[i].values[0].value;
					item.values.push_back(value);

					rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateSrv;
					updateSrv.request.knowledge.push_back(item);
					updateSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);

					if (update_knowledge_client.call(updateSrv))
						ROS_INFO("RP+: (%s) localisation accuracy is too low, deleting predicate '%s'", ros::this_node::getName().c_str(), predicate_name.c_str());
					else
						ROS_INFO("RP+: (%s) failed to update PDDL model in knowledge base", ros::this_node::getName().c_str());
				}
			}
		}
	}


	bool RPStateMonitor::remove_unused_instance(rosplan_ilm_msgs::GetInstanceCount::Request &req, rosplan_ilm_msgs::GetInstanceCount::Response &res) {
		if (req.type_name.compare("") == 0) {
			ROS_INFO("RP+: (%s) not allowed to call service with empty string as request", ros::this_node::getName().c_str());
			return false;
		}
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateSrv;
		std::vector<std::string> instances;
		get_instances_in_kb(req.type_name, instances);
		res.count = 0;
		for (std::vector<std::string>::const_iterator it = instances.begin(); it != instances.end(); ++it) {
			if ( !is_reserved_instance(*it) and count_instance_in_use(*it) == 0 ) {
				res.count++;
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				item.instance_type = req.type_name;
				item.instance_name = *it;
				updateSrv.request.knowledge.push_back(item);
				updateSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
				// ROS_INFO("RP+: (%s) about to delete instance %s of type '%s'", ros::this_node::getName().c_str(), (*it).c_str(), req.type_name.c_str());
			}
		}
		if (res.count > 0) {
			if (update_knowledge_client.call(updateSrv)) {
				ROS_INFO("RP+: (%s) deleted %d instances of type '%s'", ros::this_node::getName().c_str(), res.count, req.type_name.c_str());
				return true;
			}
			else {
				res.count = 0;
				ROS_INFO("RP+: (%s) failed to update PDDL model in knowledge base", ros::this_node::getName().c_str());
				return false;
			}
		}
		return true;
	}


	bool RPStateMonitor::set_reserved_instance(rosplan_ilm_msgs::SetReservedInstance::Request &req, rosplan_ilm_msgs::SetReservedInstance::Response &res) {
		if (req.operand == rosplan_ilm_msgs::SetReservedInstance::Request::RESET)
			reserved_instances.clear();
		if (req.operand == rosplan_ilm_msgs::SetReservedInstance::Request::RESET or req.operand == rosplan_ilm_msgs::SetReservedInstance::Request::APPEND) {
			get_instances_in_kb(req.type_name, reserved_instances);
			ROS_INFO("RP+: (%s) reserved %ld instances of type '%s'", ros::this_node::getName().c_str(), reserved_instances.size(), req.type_name.c_str());
			return true;
		}
		else if (req.operand == rosplan_ilm_msgs::SetReservedInstance::Request::TMP) {
			tmp_reserved_instances.clear();
			get_instances_in_kb(req.type_name, tmp_reserved_instances);
			ROS_INFO("RP+: (%s) temporarily reserved %ld instances of type '%s'", ros::this_node::getName().c_str(), tmp_reserved_instances.size(), req.type_name.c_str());
			return true;
		}
		else {
			ROS_INFO("RP+: (%s) unknown request operand value %d", ros::this_node::getName().c_str(), req.operand);
			return false;
		}
		// for (std::vector<std::string>::const_iterator it = reserved_instances.begin(); it != reserved_instances.end(); ++it) {
		// 	ROS_INFO("RP+: (%s) reserved instance %s", ros::this_node::getName().c_str(), (*it).c_str());
		// }
	}


	bool RPStateMonitor::is_reserved_instance(const std::string& instance) const {
		for (std::vector<std::string>::const_iterator it = reserved_instances.begin(); it != reserved_instances.end(); ++it) {
			if (instance.compare(*it) == 0)
				return true;
		}
		for (std::vector<std::string>::const_iterator it = tmp_reserved_instances.begin(); it != tmp_reserved_instances.end(); ++it) {
			if (instance.compare(*it) == 0)
				return true;
		}
		return false;
	}



} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_statemonitor");
		ros::NodeHandle nh("~");
		KCL_rosplan::RPStateMonitor rpsm(nh);
		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
