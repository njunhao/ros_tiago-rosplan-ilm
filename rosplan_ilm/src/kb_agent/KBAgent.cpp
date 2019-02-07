#include "kb_agent/KBAgent.h"


namespace KCL_rosplan {

	/* run action interface */
	void RPKBAgent::initialize_services() {

		ros::NodeHandle nh("~");

		
		// knowledge base services
		std::string kb = "knowledge_base";
		nh.getParam("knowledge_base", kb);
		std::stringstream ss;		
    	ss << "/" << kb << "/domain/operator_details";
    	ros::service::waitForService(ss.str(),ros::Duration(20));
    	query_operator_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());

    	ss.str("");
    	ss << "/" << kb << "/state/instances";
    	query_instance_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(ss.str());
    	
    	ss.str("");
    	ss << "/" << kb << "/state/propositions";
    	query_fact_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
    	
    	ss.str("");
    	ss << "/" << kb << "/domain/predicates";
    	query_predicate_kb_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(ss.str());

		ss.str("");
		ss << "/" << kb << "/update_array";
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

	}


	void RPKBAgent::get_operator_details_in_kb(const std::string op_name, rosplan_knowledge_msgs::DomainOperator& op) {
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = op_name;
		if(!query_operator_kb_client.call(srv))
			ROS_ERROR("RP+: (%s) could not query Knowledge Base for operator (%s) details", ros::this_node::getName().c_str(), op_name.c_str());
		else
			op = srv.response.op;
	}


	void RPKBAgent::get_instances_in_kb(const std::string type_name, std::vector<std::string>& instances) {
		rosplan_knowledge_msgs::GetInstanceService srv;
		srv.request.type_name = type_name;     // set to "" to get all instances
		if(!query_instance_kb_client.call(srv))
			ROS_ERROR("RP+: (%s) could not query Knowledge Base for instances of type '%s'", ros::this_node::getName().c_str(), type_name.c_str());
		else
			instances = srv.response.instances;
	}


	void RPKBAgent::get_facts_in_kb(const std::string predicate_name, std::vector<rosplan_knowledge_msgs::KnowledgeItem>& attributes) {
		rosplan_knowledge_msgs::GetAttributeService srv;
		srv.request.predicate_name = predicate_name;
		if(!query_fact_kb_client.call(srv))
			ROS_ERROR("RP+: (%s) could not query Knowledge Base for predicate '%s'", ros::this_node::getName().c_str(), predicate_name.c_str());
		else
			attributes = srv.response.attributes;
	}


	void RPKBAgent::get_domain_predicate_in_kb(std::vector<rosplan_knowledge_msgs::DomainFormula>& items) {
		rosplan_knowledge_msgs::GetDomainAttributeService srv;
		if(!query_predicate_kb_client.call(srv))
			ROS_ERROR("RP+: (%s) could not query Knowledge Base for domain predicates", ros::this_node::getName().c_str());
		else
			items = srv.response.items;
	}


	bool RPKBAgent::get_domain_predicate_params_in_kb(const std::string predicate_name, std::vector<diagnostic_msgs::KeyValue>& typed_parameters) {
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


	std::string RPKBAgent::get_key(const std::vector<diagnostic_msgs::KeyValue>& typed_parameters, const std::string typed) {
		for (uint i = 0; i < typed_parameters.size(); i++) {
			if (typed_parameters[i].value.compare(typed)==0)
				return typed_parameters[i].key;
		}
	}


	std::string RPKBAgent::get_key_of_operator(const std::string op_name, const std::string typed_instance) {
		rosplan_knowledge_msgs::DomainOperator op;
		get_operator_details_in_kb(op_name, op);
		for (size_t i = 0; i < op.formula.typed_parameters.size(); i++) {
			if (0==typed_instance.compare(op.formula.typed_parameters[i].value)) {
				ROS_INFO("---%s", op.formula.typed_parameters[i].value.c_str());
				return op.formula.typed_parameters[i].key;
			}
		}
		return std::string("");
	}


	uint RPKBAgent::count_instance_in_use(const std::string instance) {
		uint count = 0;
		rosplan_knowledge_msgs::GetAttributeService srv;
		srv.request.predicate_name = "";    // empty string will return all predicates in current state
		if(!query_fact_kb_client.call(srv))
			ROS_ERROR("RP+: (%s) could not query Knowledge Base", ros::this_node::getName().c_str());

	    // knowledge_type: 1
	    // initial_time: 
	    //   secs: 239
	    //   nsecs: 706000000
	    // is_negative: False
	    // instance_type: ''
	    // instance_name: ''
	    // attribute_name: "holding"
	    // values: 
	    //   - 
	    //     key: "r"
	    //     value: "r1"
	    //   - 
	    //     key: "o"
	    //     value: "o2"
	    // function_value: 0.0
	    // optimization: ''
	    // expr: 
	    //   tokens: []
	    // ineq: 
	    //   comparison_type: 0
	    //   LHS: 
	    //     tokens: []
	    //   RHS: 
	    //     tokens: []
	    //   grounded: False


		for (uint i = 0; i < srv.response.attributes.size(); i++) {
			if (srv.response.attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {
				for (uint j = 0; j < srv.response.attributes[i].values.size(); j++) {
					if (srv.response.attributes[i].values[j].value.compare(instance) == 0)
						count++;
				}
			}
		}
		return count;
	}


} // close namespace
