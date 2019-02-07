#ifndef REXD_CLIENT_H
#define REXD_CLIENT_H

#include <boost/asio.hpp>
#include "std_msgs/Float32.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/DispatchService.h"
#include "std_srvs/Empty.h"
#include "rosplan_ilm_msgs/GetInstanceCount.h"
#include "rosplan_ilm_msgs/SetReservedInstance.h"


using boost::asio::ip::tcp;

class rexd_client
{
public:
    rexd_client(ros::NodeHandle& nh, const std::string& host, const std::string& port);
    bool parse_command(const std::string& input_command);
    std::string get_command();

protected:
    ros::NodeHandle* node_handle;
    float last_reward;
    bool receive_problem, receive_action_dispatch, receive_action_feedback;
    double action_duration;
    std::string tmp_path, plan_path, pddl_problem_string, simulator_ppddl_current_problem_file, action_feedback;
    ros::Subscriber sub_problem, sub_actiondispatch, sub_actionfeedback;
    ros::ServiceClient parse_plan_client, generate_plan_client, dispatch_plan_client, reset_gazebo_client;
    ros::ServiceClient set_reserved_instance_client, remove_unused_instance_client;

    std::string get_state() const;
    std::string get_objects() const;
    std::string apply_action(const std::string& action);
    bool simulated_transition(std::string action, bool& success, float& rule_reward);
    bool real_transition(std::string action, bool& success, float& rule_reward);
    void send_state();
    void send_action_result(const std::string& result) const;
    void write_scenario();
    void reset_scenario(bool reset_gazebo);
    void problemTopic_callback(const std_msgs::String::ConstPtr& msg);
    void actionDispatchTopic_callback(const rosplan_dispatch_msgs::ActionDispatch& msg);
    void actionFeedbackTopic_callback(const rosplan_dispatch_msgs::ActionFeedback& msg);

private:
    tcp::iostream tcp_client;
    std::istream* client_istream;
    std::ostream* client_ostream;
};

#endif
