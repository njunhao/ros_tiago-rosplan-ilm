#include <ros/ros.h>

#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/DomainOperator.h>

#include "diagnostic_msgs/KeyValue.h"

#ifndef ilm_kb_agent
#define ilm_kb_agent

/**
 * This file defines the RPActionInterface header.
 * This header defines a standard action interface for ROSPlan.
 * This interface will link a PDDL action to some implementation, most
 * commonly as an actionlib client.
 */
namespace KCL_rosplan {

	class RPKBAgent
	{

	private:

	protected:

		/* service handle to PDDL knowledge base */
		ros::ServiceClient update_knowledge_client, query_operator_kb_client, query_instance_kb_client, query_fact_kb_client, query_predicate_kb_client;

		void initialize_services();

		void get_operator_details_in_kb(const std::string op_name, rosplan_knowledge_msgs::DomainOperator& op);

		void get_instances_in_kb(const std::string type_name, std::vector<std::string>& instances);

		void get_facts_in_kb(const std::string predicate_name, std::vector<rosplan_knowledge_msgs::KnowledgeItem>& attributes);

		void get_domain_predicate_in_kb(std::vector<rosplan_knowledge_msgs::DomainFormula>& items);

		bool get_domain_predicate_params_in_kb(const std::string predicate_name, std::vector<diagnostic_msgs::KeyValue>& typed_parameters);

		std::string get_key(const std::vector<diagnostic_msgs::KeyValue>& typed_parameters, const std::string typed);

		std::string get_key_of_operator(const std::string op_name, const std::string typed_instance);

		uint count_instance_in_use(const std::string instance);
		

	public:

	};
}
#endif
