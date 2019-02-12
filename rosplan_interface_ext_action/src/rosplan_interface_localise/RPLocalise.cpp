#include "rosplan_interface_localise/RPLocalise.h"
#include <tf/tf.h>
#include <math.h> 

#define ACCURACY 1.0
#define RAD_ACCURACY 0.174533	  // 10 deg

namespace KCL_rosplan {

	/* constructor */
	RPLocalise::RPLocalise(ros::NodeHandle &nh)
	 : message_store(nh) {

		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    	global_localization_client = nh.serviceClient<std_srvs::Empty>("/global_localization");
    	set_map_client = nh.serviceClient<nav_msgs::SetMap>("/set_map");
    	std::string topic;
    	nh.param("cmd_vel_topic", topic, std::string("/cmd_vel"));
    	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic, 1, true);
    	nh.param("pose_topic", topic, std::string("/amcl_pose"));
    	pose_sub = nh.subscribe(topic, 1, &RPLocalise::poseCallback, this);
    	nh.param("true_pose_topic", topic, std::string("/ground_truth_odom"));
    	ground_truth_pose_sub = nh.subscribe(topic, 1, &RPLocalise::truePoseCallback, this);
		nh.param("costmap_topic", topic, std::string("/move_base/global_costmap/costmap"));
		map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &RPLocalise::costMapCallback, this);
	}


	/* action dispatch callback */
	bool RPLocalise::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// #actionDispatch message
		// int32 action_id
		// string name
		// diagnostic_msgs/KeyValue[] parameters
		// float32 duration
		// float32 dispatch_time


		// ROS_INFO("KCL: (%s) waiting for localise action server to start", params.name.c_str());
		// action_client.waitForServer();
		// get current pose
		ros::spinOnce();
		if (!check_parameters(msg)) return false;

		if (simulated) {
			ROS_INFO("RP+: (%s) using simulated action, setting localisation to ground truth", params.name.c_str());
			ros::spinOnce();
			
			// clear costmaps
			std_srvs::Empty emptySrv;
			clear_costmaps_client.call(emptySrv);

			// set map for /amcl
			nav_msgs::SetMap srv;
			srv.request.map = cost_map;
			srv.request.initial_pose.header.seq = 0;
			srv.request.initial_pose.header.stamp = ros::Time::now();
			srv.request.initial_pose.header.frame_id = "map";
			srv.request.initial_pose.pose = true_pose;
			if (set_map_client.call(srv)) {
				if (srv.response.success) {
					ROS_INFO("RP+: (%s) action finished: SUCCEEDED", params.name.c_str());
					prob_effect_id = pick_probabilistic_effect();
					if (op.probabilistic_effects.size() > 0)
						ROS_INFO("RP+: (%s) %d-th probabilistic effect chosen", params.name.c_str(), prob_effect_id);
					return true;
				}
				else {
					ROS_INFO("RP+: (%s) action finished: FAILED", params.name.c_str());
					prob_effect_id = 1;
					return false;
				}
			}
			else {
				ROS_INFO("RP+: (%s) failed to call service '/set_map'", params.name.c_str());
				prob_effect_id = 1;
				return false;
			}
		}

		// populate particles in map
		std_srvs::Empty emptySrv;
		global_localization_client.call(emptySrv);

		geometry_msgs::Twist cmd_vel;
		ros::Time start_time = ros::Time::now();
		ros::Duration timeout(60); // Timeout in seconds
		ros::Rate r(10); // 10 hz

		double initial_yaw = convert2Euler(pose.pose);

		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.8;

		bool half_turn = false;
    	while((ros::Time::now() - start_time) < timeout) {
    		ros::spinOnce();   // get updated pose
    		double current_yaw = convert2Euler(pose.pose);
    		// ROS_INFO("Current status: %f and %f", current_yaw, initial_yaw);
    		if (fabs(current_yaw - initial_yaw) >= 135.0) {   // do not set 180 due to r.sleep()
    			half_turn = true;
    			// ROS_INFO("Half turn completed");
    		}
			if ((fabs(current_yaw - initial_yaw) < 15.0) and half_turn) {
				// ROS_INFO("Full turn completed");
				break;
			}
			cmd_vel_pub.publish(cmd_vel);
			r.sleep();
		}

		
		if (false) {
			if (!accurate_localisation()) {
				// start_time = ros::Time::now();
				// cmd_vel.linear.x = 1.0;
				// cmd_vel.angular.z = 0.0;
				// ros::Duration timeout2(5); // Timeout in seconds
				// while((ros::Time::now() - start_time) < timeout2) {
				// 	cmd_vel_pub.publish(cmd_vel);
				// 	r.sleep();
				// }
				// start_time = ros::Time::now();
				// initial_yaw = convert2Euler(pose.pose.pose);
				// cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = -0.8;

				start_time = ros::Time::now();
				half_turn = false;
	    		while((ros::Time::now() - start_time) < timeout) {
		    		ros::spinOnce();   // get updated pose
		    		double current_yaw = convert2Euler(pose.pose);
		    		// ROS_INFO("Current status: %f and %f", current_yaw, initial_yaw);
		    		if (fabs(current_yaw - initial_yaw) >= 135.0) {   // do not set 180 due to r.sleep()
		    			half_turn = true;
		    			// ROS_INFO("Half turn completed");
		    		}
					if ((fabs(current_yaw - initial_yaw) < 15.0) and half_turn) {
						// ROS_INFO("Full turn completed");
						break;
					}
					cmd_vel_pub.publish(cmd_vel);
					r.sleep();
				}
			}
		}
		else {
			// use ground truth as localised pose
			ros::spinOnce();   // suppose to trigger costMapCallback but didn't
			ROS_INFO("RP+: (%s) using ground truth as localised pose", params.name.c_str());
			// clear costmaps
			// std_srvs::Empty emptySrv;
			// clear_costmaps_client.call(emptySrv);

			// set map for /amcl
			nav_msgs::SetMap srv;
			srv.request.map = cost_map;
			srv.request.initial_pose.header.seq = 0;
			srv.request.initial_pose.header.stamp = ros::Time::now();
			srv.request.initial_pose.header.frame_id = "map";
			srv.request.initial_pose.pose = true_pose;
			if (set_map_client.call(srv)) {
				if (srv.response.success) {
					ROS_INFO("RP+: (%s) action finished: SUCCEEDED", params.name.c_str());
					prob_effect_id = 0;
					if (op.probabilistic_effects.size() > 0)
						ROS_INFO("RP+: (%s) %d-th probabilistic effect chosen", params.name.c_str(), prob_effect_id);
					return true;
				}
				else {
					ROS_INFO("RP+: (%s) action finished: FAILED", params.name.c_str());
					prob_effect_id = 1;
					return false;
				}
			}
			else {
				ROS_INFO("RP+: (%s) failed to call service '/set_map'", params.name.c_str());
				prob_effect_id = 1;
				return false;
			}
		}

		if (accurate_localisation()) {
			ROS_INFO("RP+: (%s) action finished with low variance in position", params.name.c_str());
			prob_effect_id = 0;
		}
		else {
			ROS_INFO("RP+: (%s) action finished with high variance in position", params.name.c_str());
			prob_effect_id = 1;
		}

		return true;
	}


	// void RPLocalise::poseCallback(const nav_msgs::Odometry& msg) {
	// 	pose = msg.pose;
 //     	ROS_INFO("RP+: Received current pose");
	// }


	// void RPLocalise::motion(const std::string motion, float duration) {
	// 	// rotate on the spot
	// 	geometry_msgs::Twist cmd_vel;
	// 	ros::Time start_time = ros::Time::now();
	// 	ros::Duration timeout(duration); // Timeout in seconds
	// 	ros::Rate r(10); // 10 hz
	// 	if (motion.compare("rotate")==0) {
	// 		// CANNOT USE THIS ANYMORE AS IT REQUIRES spinOnce which will then trigger concreteCallback resulting in circular function calls
	// 		double initial_yaw = convert2Euler(pose.pose.pose);

	// 		cmd_vel.linear.x = 0.0;
	// 		cmd_vel.linear.y = 0.0;
	// 		cmd_vel.linear.z = 0.0;
	// 		cmd_vel.angular.x = 0.0;
	// 		cmd_vel.angular.y = 0.0;
	// 		cmd_vel.angular.z = 1.0;

	// 		bool half_turn = false;
	//     	while((ros::Time::now() - start_time) < timeout) {
	//     		ros::spinOnce();   // get updated pose
	//     		double current_yaw = convert2Euler(pose.pose);
	//     		// ROS_INFO("Current status: %f and %f", current_yaw, initial_yaw);
	//     		if (fabs(current_yaw - initial_yaw) >= 135.0) {   // do not set 180 due to r.sleep()
	//     			half_turn = true;
	//     			// ROS_INFO("Half turn completed");
	//     		}
	// 			if ((fabs(current_yaw - initial_yaw) < 15.0) and half_turn) {
	// 				// ROS_INFO("Full turn completed");
	// 				break;
	// 			}
	// 			cmd_vel_pub.publish(cmd_vel);
	// 			r.sleep();
	// 		}
	// 		cmd_vel.linear.x = 0.0;
	// 		cmd_vel.linear.y = 0.0;
	// 		cmd_vel.linear.z = 0.0;
	// 		cmd_vel.angular.x = 0.0;
	// 		cmd_vel.angular.y = 0.0;
	// 		cmd_vel.angular.z = 1.0;
	// 	} else if (motion.compare("forward")==0) {
	// 		cmd_vel.linear.x = 1.0;
	// 		cmd_vel.linear.y = 0.0;
	// 		cmd_vel.linear.z = 0.0;
	// 		cmd_vel.angular.x = 0.0;
	// 		cmd_vel.angular.y = 0.0;
	// 		cmd_vel.angular.z = 0.0;
	// 	}
 //    	while((ros::Time::now() - start_time) < timeout) {
	// 		cmd_vel_pub.publish(cmd_vel);
	// 		r.sleep();
	// 	}
	// }


	bool RPLocalise::accurate_localisation() const {
		return ((sqrt(fabs(pose.covariance[0])) < ACCURACY) 
				and (sqrt(fabs(pose.covariance[7])) < ACCURACY) 
				and (sqrt(fabs(pose.covariance[14])) < ACCURACY)
				and (sqrt(fabs(pose.covariance[35])) < RAD_ACCURACY));
	}


	void RPLocalise::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
		pose = msg.pose;
     	// ROS_INFO("RP+: Received current pose");
	}


	void RPLocalise::truePoseCallback(const nav_msgs::Odometry& msg) {
		true_pose = msg.pose;
	}


	void RPLocalise::costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg ) {
		cost_map = *msg;
	}


	double convert2Euler(const geometry_msgs::Pose& pose) {	    
	    tf::Quaternion q(
	         pose.orientation.x,
	         pose.orientation.y,
	         pose.orientation.z,
	         pose.orientation.w);
	    tf::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
	    return yaw*180/PI;    // return in degrees
	}



} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_localise");
		ros::NodeHandle nh("~");
		// create PDDL action subscriber
		KCL_rosplan::RPLocalise rplc(nh);
		rplc.runActionInterface();

		return 0;
	}
