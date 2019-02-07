#include <ros/ros.h>
#include <boost/tokenizer.hpp>

#include "kb_agent/KBAgent.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"

#ifndef KCL_prob_action_interface
#define KCL_prob_action_interface

/**
 * This file defines the RPActionInterface header.
 * This header defines a standard action interface for ROSPlan.
 * This interface will link a PDDL action to some implementation, most
 * commonly as an actionlib client.
 */
namespace KCL_rosplan {

	class RPProbActionInterface: public RPKBAgent
	{

	private:

	protected:

		/* PDDL info and publisher */
		std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates;
		rosplan_knowledge_msgs::DomainFormula params;
		rosplan_knowledge_msgs::DomainOperator op;
		ros::Publisher pddl_action_parameters_pub;
		std::vector<std::string> pddl_params;

		/* action feedback to planning system */
		ros::Publisher action_feedback_pub;

		/* action status */
		bool action_success;

		/* index of probabilistic effect that took effect */
		uint prob_effect_id;

		/* if true, simulate action */
		bool simulated;


		bool check_parameters(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		uint pick_probabilistic_effect() const;

	public:

		/* main loop for action interface */
		void runActionInterface();

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		/* perform or call real action implementation */
		virtual bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) =0;
	};
}
#endif
