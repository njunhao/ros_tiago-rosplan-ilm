#include "rosplan_interface_explore/RPExplore.h"
#include <stdlib.h>     /* atof */

namespace KCL_rosplan {

	/* constructor */
	RPExplore::RPExplore(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), action_client(actionserver, true) {  // pass in true in constructor of action_client to allow multi-threading
		initialize(nh);
	}

	/* action dispatch callback */
	bool RPExplore::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ros::spinOnce();
		if (!check_parameters(msg)) return false;
		start_process();

		if (simulated) {
			ROS_INFO("RP+: (%s) using simulated action", params.name.c_str());
			bool success = run_simulated_action();
			if (success) {
				ROS_INFO("RP+: (%s) action finished: SUCCEEDED", params.name.c_str());
				return true;
			}
			else {
				ROS_INFO("RP+: (%s) action finished: FAILED", params.name.c_str());
				return false;
			}
		}

		if (exploration_path.poses.size() == 0) {
			ROS_INFO("RP+: (%s) no waypoints received, using preset waypoints for exploration", params.name.c_str());
			if (!parse_file(exploration_path)) {
				ROS_INFO("RP+: (%s) failed to parse file '%s'", params.name.c_str(), data_path.c_str());
				return false;
			}
		}


		ROS_INFO("RP+: (%s) waiting for move_base action server to start", params.name.c_str());
		action_client.waitForServer();
		for (uint i = 0; i < exploration_path.poses.size(); i++) {
			ROS_INFO("RP+: (%s) going to waypoint %d (x: %f, y: %f, z: %f)", params.name.c_str(), i, 
				exploration_path.poses[i].pose.position.x, exploration_path.poses[i].pose.position.y, exploration_path.poses[i].pose.position.z);

			// dispatch MoveBase action
			move_base_msgs::MoveBaseGoal goal;
			geometry_msgs::PoseStamped &pose = exploration_path.poses[i];
			goal.target_pose = pose;
			action_client.sendGoal(goal);
			
			bool finished_before_timeout = false;
			while (ros::ok() and continue_exploring() and !finished_before_timeout) {
				finished_before_timeout = action_client.waitForResult(ros::Duration(0.1));
				ros::spinOnce();   // check if still need to explore
			}			

			if (finished_before_timeout) {
				if (action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
					// clear costmaps
					std_srvs::Empty emptySrv;
					clear_costmaps_client.call(emptySrv);

					// publish feedback (failed)
					return false;
				}
			} else {
				// timed out or received request to terminate (failed)
				action_client.cancelAllGoals();
				if (!continue_exploring()) {
					end_process();
					if (is_successful())
						ROS_INFO("RP+: (%s) action finished: SUCCEEDED", params.name.c_str());
					else
						ROS_INFO("RP+: (%s) action cancelled", params.name.c_str());
					return is_successful();   // let derived class determine if action has succeeded
				}
				else {
					end_process();
					ROS_INFO("RP+: (%s) action timed out", params.name.c_str());
					return false;
				}
			}
		}
		end_process();
		actionlib::SimpleClientGoalState state = action_client.getState();
		ROS_INFO("RP+: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());
		// publish feedback (achieved)
		return true;
	}


	void RPExplore::explorationPathCallback(const nav_msgs::Path &expath) {
		exploration_path = expath;
	}


	void RPExplore::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
		robot_pose = msg;
		// double roll, pitch, yaw;
		// // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
		// tf::Quaternion quat;
		// tf::quaternionMsgToTF(robot_pose.pose.pose.orientation, quat);
		// tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		// ROS_INFO("RP+: (%s) : robot pose is x: %f, y: %f, z: %f, q.x: %f, q.y: %f, q.z: %f, q.w: %f, roll: %f, pitch: %f, yaw: %f", params.name.c_str(), 
		// 	robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, robot_pose.pose.pose.position.z,
		// 	robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y, robot_pose.pose.pose.orientation.z, robot_pose.pose.pose.orientation.w,
		// 	roll, pitch, yaw);
	}


	void RPExplore::initialize(ros::NodeHandle &nh) {
		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
		global_planner_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
		std::string topic;
		nh.param("pose_topic", topic, std::string("/amcl_pose"));
	    sub_pose = nh.subscribe(topic, 1, &RPExplore::poseCallback, this);
		nh.param("exploration_path_topic", topic, std::string("/exploration_path"));
		exploration_path_sub =	 nh.subscribe(topic, 1, &RPExplore::explorationPathCallback, this);
		nh.param("preset_waypoints", data_path, std::string("/"));
		prob_effect_id = 100;  // dummy value, if this is > num of probabilistic effects, no probabilistic effect will be effected in KB
	}


	void RPExplore::start_process() {
		nav_msgs::Path empty_path;
		exploration_path = empty_path;
	}


	void RPExplore::end_process() {
		// do nothing
	}


	bool RPExplore::run_simulated_action() {
		prob_effect_id = pick_probabilistic_effect();
		if (op.probabilistic_effects.size() > 0)
			ROS_INFO("RP+: (%s) %d-th probabilistic effect chosen", params.name.c_str(), prob_effect_id);
		return true;
	}


	void RPExplore::set_prob_effect() {
		// do nothing
	}


	bool RPExplore::continue_exploring() {
		return true;
	}


	bool RPExplore::is_successful() {
		return false;
	}


	bool RPExplore::parse_file(nav_msgs::Path &expath) {
		bool DEBUG = false;
		std::ifstream infile(data_path.c_str());   // load file
		if (infile.fail()) return false;
		ROS_INFO("RP+: (%s) reading file '%s' for waypoints", params.name.c_str(), data_path.c_str());
		std::string line, frame_id;
		int seq = 0, line_id = 0;

		std::vector<geometry_msgs::Pose> waypoints;
		while (std::getline(infile, line)) {
			line_id++;
	 		if (line.substr(0,1).compare("#")==0) continue;   // ignore comments
			int curr, next = 0;
			curr = line.find("frame_id=",next);
			if (curr != std::string::npos)
				frame_id = line.substr(curr+9, 100);   // till end
			else {
				if (DEBUG) ROS_INFO("%s",frame_id.c_str());

				geometry_msgs::Pose waypoint;
				
				curr = line.find("x=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 2;
				next = line.find(";", curr);
				waypoint.position.x = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

				curr = line.find("y=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 2;
				next = line.find(";", curr);
				waypoint.position.y = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

				curr = line.find("z=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 2;
				next = line.find(";", curr);
				waypoint.position.z = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

				double r=0, p=0, y=0;  // Rotate the previous pose by 180* about X
				
				curr = line.find("roll=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 5;
				next = line.find(";", curr);
				r = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

				curr = line.find("pitch=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 6;
				next = line.find(";", curr);
				p = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

				curr = line.find("yaw=");
				if (curr == std::string::npos) {
					ROS_INFO("RP+: (%s) line %d is malformed", params.name.c_str(), line_id);
					continue;
				}
				curr = curr + 4;
				next = line.find(";", curr);
				y = atof(line.substr(curr, next-curr).c_str());
				if (DEBUG) ROS_INFO("%s",line.substr(curr, next-curr).c_str());

		   		tf::Quaternion q = tf::createQuaternionFromRPY(r, p, y);
				waypoint.orientation.x = q[0];
				waypoint.orientation.y = q[1];
				waypoint.orientation.z = q[2];
				waypoint.orientation.w = q[3];
				waypoints.push_back(waypoint);
			}
		}
		infile.close();

		if (waypoints.size() == 0) {
			ROS_INFO("RP+: (%s) no waypoints found", params.name.c_str());
			return false;
		}
		else {
			expath.header.seq = 1;
			expath.header.stamp = ros::Time::now();
			expath.header.frame_id = frame_id;
			greedy_path_planning(waypoints, frame_id);
			for (uint i = 0; i < waypoints.size(); i++) {
				geometry_msgs::PoseStamped waypoint;
				waypoint.header.seq = i+1;
				waypoint.header.frame_id = frame_id;
				// waypoint.header.stamp.secs = 0;
				// waypoint.header.stamp.nsecs = 0;
				waypoint.pose = waypoints[i];
				expath.poses.push_back(waypoint);
			}
			return true;
		}
	}


	void RPExplore::greedy_path_planning(std::vector<geometry_msgs::Pose>& waypoints, const std::string frame_id) {
		std::vector<uint> added(waypoints.size(), 0);
		std::vector<geometry_msgs::Pose> new_waypoints;
		geometry_msgs::Pose start_pose = robot_pose.pose.pose;
		// ROS_INFO("robot x: %f y: %f", robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y);
		for (uint i = 0; i < waypoints.size(); i++) {
			double min_cost = 999999;
			uint min_id;
			for (uint j = 0; j < waypoints.size(); j++) {
				if (added[j] > 0) continue;
				double cost = get_path_cost(start_pose, waypoints[j], frame_id);
				if (cost < min_cost) {
					min_cost = cost;
					min_id = j;
				}
			}
			added[min_id] = 1;
			start_pose = waypoints[min_id];
			new_waypoints.push_back(waypoints[min_id]);
			// ROS_INFO("x: %f y: %f", waypoints[min_id].position.x, waypoints[min_id].position.y);
		}
		waypoints = new_waypoints;
	}


	double RPExplore::get_path_cost(geometry_msgs::Pose wp1, geometry_msgs::Pose wp2, const std::string frame_id) {
		nav_msgs::GetPlan srv;
		srv.request.start.header.frame_id = frame_id;
		srv.request.start.pose = wp1;
		srv.request.goal.header.frame_id = frame_id;
		srv.request.goal.pose = wp2;
		srv.request.tolerance = 2;
		if (global_planner_client.call(srv)) {
			double cost = 0.0;
			for (uint i = 1; i < srv.response.plan.poses.size(); i++)
				cost += euclidean_distance(srv.response.plan.poses[i-1].pose, srv.response.plan.poses[i].pose);
			return cost;
		}
		else
			return euclidean_distance(wp1, wp2);
	}


	double RPExplore::euclidean_distance(geometry_msgs::Pose wp1, geometry_msgs::Pose wp2) {
		return sqrt(pow(wp1.position.x-wp2.position.x,2) + pow(wp1.position.y-wp2.position.y, 2));
	}



} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "explore");
		ros::NodeHandle nh("~");

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/explore"));

		// create PDDL action subscriber
		KCL_rosplan::RPExplore rpex(nh, actionserver);

		rpex.runActionInterface();

		return 0;
	}