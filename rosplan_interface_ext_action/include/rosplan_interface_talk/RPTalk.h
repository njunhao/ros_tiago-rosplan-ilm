#include <ros/ros.h>
#include <vector>

#include "ActionInterface/RPProbActionInterface.h"
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include "actionlib/client/simple_action_client.h"
// #include "mongodb_store/message_store.h"


#ifndef ext_talk
#define ext_talk


namespace KCL_rosplan {

	template <class actionType, class actionMsg, class detectionMsg>
	class RPTalk: public RPProbActionInterface
	{

	protected:
		ros::Subscriber sub_detector;
		ros::ServiceClient update_kb_client, query_instance_kb_client, query_fact_kb_client, query_predicate_kb_client;
		// mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<actionType> action_client;
		detectionMsg detections;

		virtual void detectionCallback(const detectionMsg& msg) = 0;
		virtual void start_process() = 0;
		virtual void end_process(bool success) = 0;
		virtual void get_action_msg(actionMsg& action_goal) = 0;
		virtual bool facing_target() = 0;
		virtual bool run_simulated_action() = 0;

		virtual void initialize(ros::NodeHandle &nh) {
			std::string topic;
			// nh.param("pose_topic", topic, std::string("/amcl_pose"));
			// sub_pose = nh.subscribe(topic, 1, &RPTalk::poseCallback, this);
	    	nh.param("detector_topic", topic, std::string("/detections"));
	    	sub_detector = nh.subscribe(topic, 1, &RPTalk::detectionCallback, this);
	    	// nh.param("depth_image_topic", topic, std::string("/image_raw"));
	    	// sub_image = nh.subscribe(topic, 1, &RPTalk::depthImageCallback, this);
			
			nh.param("knowledge_base", topic, std::string("knowledge_base"));
			std::stringstream ss;
			ss << "/" << topic << "/update_array";
	    	update_kb_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());
	    	ss.str(std::string());   // clear stringstream
	    	ss << "/" << topic << "/state/instances";
	    	query_instance_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(ss.str());
	    	ss.str(std::string());
	    	ss << "/" << topic << "/state/propositions";
	    	query_fact_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
	    	ss.str(std::string());
	    	ss << "/" << topic << "/domain/predicates";
	    	query_predicate_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(ss.str());

			prob_effect_id = 100;  // dummy value, if this is > num of probabilistic effects, no probabilistic effect will be effected in KB
		}


		void update_kb(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray& srv) {
			if(!update_kb_client.call(srv))
				ROS_ERROR("RP+: (%s) could not update Knowledge Base", ros::this_node::getName().c_str());
		}


		void get_instances_in_kb(const std::string type_name, std::vector<std::string>& instances) {
			rosplan_knowledge_msgs::GetInstanceService srv;
			srv.request.type_name = type_name;
			if(!query_instance_kb_client.call(srv))
				ROS_ERROR("RP+: (%s) could not query Knowledge Base for instances of type '%s'", ros::this_node::getName().c_str(), type_name.c_str());
			else
				instances = srv.response.instances;
		}


		void get_facts_in_kb(const std::string predicate_name, std::vector<rosplan_knowledge_msgs::KnowledgeItem>& attributes) {
			rosplan_knowledge_msgs::GetAttributeService srv;
			srv.request.predicate_name = predicate_name;
			if(!query_fact_kb_client.call(srv))
				ROS_ERROR("RP+: (%s) could not query Knowledge Base for predicate '%s'", ros::this_node::getName().c_str(), predicate_name.c_str());
			else
				attributes = srv.response.attributes;
		}


		void get_domain_predicate_in_kb(std::vector<rosplan_knowledge_msgs::DomainFormula>& items) {
			rosplan_knowledge_msgs::GetDomainAttributeService srv;
			if(!query_predicate_kb_client.call(srv))
				ROS_ERROR("RP+: (%s) could not query Knowledge Base for domain predicates", ros::this_node::getName().c_str());
			else
				items = srv.response.items;
		}


		bool get_domain_predicate_params_in_kb(const std::string predicate_name, std::vector<diagnostic_msgs::KeyValue>& typed_parameters) {
			std::vector<rosplan_knowledge_msgs::DomainFormula> items;
			get_domain_predicate_in_kb(items);
			for (uint i = 0; i < items.size(); i++) {
				if (items[i].name.compare(predicate_name)==0) {
					typed_parameters = items[i].typed_parameters;
					return true;
				}
			}
			return false;
		}


	public:

		/* constructor */
		RPTalk(ros::NodeHandle &nh, std::string &actionserver)
		 : action_client(actionserver, true) {  // pass in true in constructor of action_client to allow multi-threading
			initialize(nh);
		}

		/* listen to and process action_dispatch topic */
		// set to virtual so that function uses objects in derived class
		/* action dispatch callback */
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			start_process();
			ros::spinOnce();
			if (!check_parameters(msg) or !facing_target()) return false;

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
			
			ROS_INFO("RP+: (%s) waiting for action server to start", params.name.c_str());
			action_client.waitForServer();
			// dispatch action
			actionMsg action_goal;
			get_action_msg(action_goal);
			action_client.sendGoal(action_goal);

			bool finished_before_timeout = action_client.waitForResult();
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

				if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
					end_process(true);
					actionlib::SimpleClientGoalState state = action_client.getState();
					ROS_INFO("RP+: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());
					// publish feedback (achieved)
					return true;

				} else {
					end_process(false);
					// publish feedback (failed)
					return false;
				}
			} else {
				// timed out (failed)
				end_process(false);
				action_client.cancelAllGoals();
				ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
				return false;
			}
		}

	};

}
#endif
