#include <ros/ros.h>
#include <string.h>
#include "std_msgs/String.h"
#include "TCP/rexd_client.h"
#include <algorithm>
#include <iostream>
#include <fstream>

#define WAYPOINT "waypoint"


rexd_client::rexd_client(ros::NodeHandle& nh, const std::string& host, const std::string& port) {
    node_handle = &nh;

    tcp_client.connect(host, port);
    if (!tcp_client) {
        ROS_ERROR("RP+ (TCP): unable to connect to host %s on port %s",host.c_str(), port.c_str());
        ros::Duration(2.0).sleep();
        ros::shutdown();
        return;
    }
    ROS_INFO("RP+ (TCP): connected to host %s...", host.c_str());
    // this code doesn't work
    // ros::Rate r(1); // 1 hz
    // while (ros::ok() and !tcp_client) {
    //     ROS_INFO("RP+ (TCP): unable to connect to host %s on port %s",host.c_str(), port.c_str());
    //     tcp_client.connect(host, port);
    //     r.sleep();
    // }
    
    reset_scenario(false);   // write PPDDL domain and problem and update KB
    std::string problemTopic, plannerTopic, actionDispatchTopic, actionFeedbackTopic;
    node_handle->getParam("problem_topic", problemTopic);
    node_handle->getParam("action_dispatch_topic", actionDispatchTopic);
    node_handle->getParam("action_feedback_topic", actionFeedbackTopic);
    node_handle->getParam("data_path", plan_path);
    tmp_path = plan_path + "tmp.txt";
    plan_path = plan_path + "plan.pddl";
    simulator_ppddl_current_problem_file = "problem_current.pddl";  // this must match the value in rexd_config.cfg

    sub_problem = node_handle->subscribe(problemTopic, 1, &rexd_client::problemTopic_callback, this);
    sub_actiondispatch = node_handle->subscribe(actionDispatchTopic, 1, &rexd_client::actionDispatchTopic_callback, this);
    sub_actionfeedback = node_handle->subscribe(actionFeedbackTopic, 1, &rexd_client::actionFeedbackTopic_callback, this);
    // $ rosservice type /rosplan_parsing_interface/parse_plan_from_file
    // rosplan_dispatch_msgs/ParsingService
    // The above line is the location of the .srv file that goes into <rosplan_dispatch_msgs::ParsingService>
    // $ rosservice type /rosplan_parsing_interface/parse_plan_from_file  | rossrv show
    //      string plan_path
    //      ---
    //      bool plan_parsed
    // the string in "" are the service being advertised as, found using $ rosservice list | grep parse_plan_from_file
    parse_plan_client = node_handle->serviceClient<rosplan_dispatch_msgs::ParsingService>("/rosplan_parsing_interface/parse_plan_from_file");
    generate_plan_client = node_handle->serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    dispatch_plan_client = node_handle->serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
    reset_gazebo_client = node_handle->serviceClient<std_srvs::Empty>("/gazebo/reset_world");   // do not use reset_simulation which resets sim_time and causes other nodes to malfunction
    remove_unused_instance_client = node_handle->serviceClient<rosplan_ilm_msgs::GetInstanceCount>("/rosplan_state_monitor/remove_unused_instance");
    set_reserved_instance_client = node_handle->serviceClient<rosplan_ilm_msgs::SetReservedInstance>("/rosplan_state_monitor/set_reserved_instance");

    client_istream = &tcp_client;
    client_ostream = &tcp_client;
    std::cout << "Connected!" << std::endl;

    // set instances in problem.pddl as reserved instances which will not be deleted
    rosplan_ilm_msgs::SetReservedInstance srv;
    srv.request.operand = rosplan_ilm_msgs::SetReservedInstance::Request::RESET;
    srv.request.type_name = "";
    if (!set_reserved_instance_client.call(srv))
        ROS_ERROR("RP+ (TCP): failed to call service /rosplan_state_monitor/set_reserved_instance");
}


bool rexd_client::parse_command(const std::string& input_command) {
    if (input_command.size() >= 3)
        ROS_INFO("RP+ (TCP): server command is: %s", input_command.c_str());
    if (input_command.size() < 3) {
        // ROS_INFO("RP+ (TCP): Ignoring too short command");
    }
    else if (input_command.find("request_state")!=std::string::npos) {
        send_state();
    }
    else if (input_command.find("planning_started")!=std::string::npos) {
         ROS_INFO("RP+ (TCP): planning started");
    }
    else if (input_command.find("planning_finished")!=std::string::npos) {
         ROS_INFO("RP+ (TCP): planning finished");
    }
    else if (input_command.find("reset")!=std::string::npos) {
        reset_scenario(true);
        // set instances in problem.pddl as reserved instances which will not be deleted
        rosplan_ilm_msgs::SetReservedInstance srv;
        srv.request.type_name = "";
        srv.request.operand = rosplan_ilm_msgs::SetReservedInstance::Request::RESET;
        if (!set_reserved_instance_client.call(srv))
            ROS_ERROR("RP+ (TCP): failed to call service /rosplan_state_monitor/set_reserved_instance");
    }
    else if (input_command.find("finish_simulator")!=std::string::npos) {
        ROS_INFO("RP+ (TCP): finishing simulation...");
        return false;
    }
    else {
        std::string result;
        ROS_INFO("RP+ (TCP): received action ");
        result = apply_action(input_command);
        send_action_result(result);

        // delete unused instances of type waypoint
        rosplan_ilm_msgs::GetInstanceCount removeSrv;
        removeSrv.request.type_name = WAYPOINT;
        if (!remove_unused_instance_client.call(removeSrv))
            ROS_ERROR("RP+ (TCP): failed to call service /rosplan_state_monitor/remove_unused_instance");

        // set current state instances as temporary reserved instances else if these are deleted, then rex-d will not be able to find
        // the instance when creating the state transition set. the missing instance will then be set as type 'default' --> exploitation will fail
        // this must be called after deleting unused instances
        rosplan_ilm_msgs::SetReservedInstance setSrv;
        setSrv.request.type_name = "";
        setSrv.request.operand = rosplan_ilm_msgs::SetReservedInstance::Request::TMP;
        if (!set_reserved_instance_client.call(setSrv))
            ROS_ERROR("RP+ (TCP): failed to call service /rosplan_state_monitor/set_reserved_instance");
    }
    return true;
}


std::string rexd_client::get_state() const {
    // ROS_INFO("RP+ (TCP): getting state...");
    std::string line;
    std::ifstream myfile(tmp_path);
    if (myfile.is_open()) {
        std::getline(myfile,line);
        // ROS_INFO("RP+ (TCP): state is: %s", line.c_str());
        myfile.close();
    }

    // ros::Time start_time = ros::Time::now();
    // ros::Duration timeout(20.0); // Timeout in seconds
    // while ((ros::Time::now() - start_time) < timeout) {
    //     ros::spinOnce();
    // }

    return line;
    // return "-hasspare() notFlattire() road(l11 - location l21 - location) road(l12 - location l11 - location) road(l12 - location l22 - location) road(l13 - location l12 - location) road(l13 - location l22 - location) road(l21 - location l31 - location) road(l22 - location l21 - location) road(l22 - location l31 - location) spareIn(l11 - location) spareIn(l12 - location) spareIn(l21 - location) vehicleAt(l13 - location)";
}


std::string rexd_client::get_objects() const {
    // ROS_INFO("RP+ (TCP): getting objects...");
    std::string line;
    std::ifstream myfile(tmp_path);
    if (myfile.is_open()) {
        std::getline(myfile,line);
        std::getline(myfile,line);   // get 2nd line
        // ROS_INFO("RP+ (TCP): objects are: %s", line.c_str());
        myfile.close();
    }
    return line;
    // return "(:objects l11 l12 l13 l21 l22 l31 - location  )";
}


bool rexd_client::simulated_transition(std::string action, bool& success, float& rule_reward) {
    success = false;
    rule_reward = 0.0;
    return true;
}


bool rexd_client::real_transition(std::string action, bool& success, float& rule_reward) {
    bool applied = false;
    success = false;
    rule_reward = 0.0;
    receive_action_dispatch = false;
    receive_action_feedback = false;

    // *** write action to plan.pddl ***
    std::string parser_path;
    node_handle->getParam("parser_path", parser_path);
    std::string command = "python3 " + parser_path + " write_plan " + plan_path + " \"" + action + "\"";
    system(command.c_str());

    // *** call service to parse plan ***
    // action from rexd: moveCar(l12, l31)
    // Plan for ROSPlan:
    // 0.000: (undock kenny wp1)  [10.000]
    // 10.001: (localise kenny)  [60.000]
    // rosservice call /rosplan_parsing_interface/parse_plan_from_file "plan_path: '[...]/plan_tutorial_03.pddl'"
    rosplan_dispatch_msgs::ParsingService parseSrv;
    parseSrv.request.plan_path = plan_path;
    if (parse_plan_client.call(parseSrv)) {
        ROS_INFO("RP+ (TCP): calling service /rosplan_parsing_interface/parse_plan_from_file");
        if (parseSrv.response.plan_parsed) {
            // now call service to dispatch plan
            rosplan_dispatch_msgs::DispatchService dispatchSrv;
            if (!dispatch_plan_client.call(dispatchSrv))
                ROS_ERROR("RP+ (TCP): failed to call service /rosplan_plan_dispatcher/dispatch_plan");
        }
    }
    else
        ROS_ERROR("RP+ (TCP): failed to call service /rosplan_parsing_interface/parse_plan_from_file");

    // *** get dispatch feedback ***
    // rosplan_dispatch_msgs/ActionDispatch
    //  int32 action_id
    //  string name
    //  diagnostic_msgs/KeyValue[] parameters
    //    string key
    //    string value
    //  float32 duration
    //  float32 dispatch_time
    // DO NOTHING

    // *** get action feedback ***
    // rosplan_dispatch_msgs/ActionFeedback
    //  int32 action_id
    //  string status
    //  diagnostic_msgs/KeyValue[] information
    //    string key
    //    string value
    // $ rostopic echo /rosplan_plan_dispatcher/action_feedback -n 1
    //      action_id: 1
    //      status: action achieved
    //      information: []
    
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(3.0); // Timeout in seconds
    while ((ros::Time::now() - start_time) < timeout) {
        // ROS_INFO("RP+ (TCP): waiting for action dispatch...");
        ros::spinOnce();
        if (receive_action_dispatch)
            break;
    }
    if (!receive_action_dispatch) action_duration = 10;  // did not receive duration, set to default

    start_time = ros::Time::now();
    ros::Duration timeout2(action_duration+10.0); // Timeout in seconds
    while ((ros::Time::now() - start_time) < timeout2) {
        // ROS_INFO("RP+ (TCP): waiting for action feedback...");
        ros::spinOnce();
        if (receive_action_feedback)
            break;
    }

    start_time = ros::Time::now();
    while ((ros::Time::now() - start_time) < timeout2 and receive_action_feedback) {
        // ROS_INFO("RP+ (TCP): waiting for next action feedback...");
        ros::spinOnce();
        if(0 == action_feedback.compare("precondition false")) {
            applied = false;
            break;
        }
        // action enabled (precondition is true)
        if(0 == action_feedback.compare("action enabled") or 0 == action_feedback.compare("action dispatched"))
            applied = true;

        // action completed (successfuly)
        if(0 == action_feedback.compare("action achieved")) {
            applied = true;
            success = false;    // not in use by rex-d, just set as false
            break;
        }

        // action completed (failed)
        if(0 == action_feedback.compare("action failed")) {
            applied = true;
            success = false;   // assume action fail to complete because of unknown factors not due to unsatisfied precondition
            break;
        }
    }
    return applied;
}


std::string rexd_client::apply_action(const std::string& action) {
    bool success, applied;      // applied is true if action is executed completely, success is true if effect of highest probability takes place (NOT USED)
    float rule_reward;

    applied = real_transition(action, success, rule_reward);
    last_reward = rule_reward;
    std::string action_result;
    if (applied) {
        if (success) {
            // applied successfully"
            action_result = "success";
        }
        else {
            // applied NOT successfully
            action_result = "applied_not_success";
        }
    }
    else {
        // couldn't be applied
        action_result = "failed";
    }
    return action_result;
}


void rexd_client::reset_scenario(bool reset_gazebo) {            // rexd loads next planning problem
    ROS_INFO("RP+ (TCP): resetting scenario");
    write_scenario();
    system("rosnode kill /rosplan_knowledge_base");
    system("rosnode kill /depthimage_to_laserscan");
    ros::Duration(5.0).sleep();  // give KB time to initialize again
    // reset Gazebo and rviz
    if (reset_gazebo) {
        std_srvs::Empty srv;
        if (reset_gazebo_client.call(srv))
            ROS_INFO("RP+ (TCP): resetting Gazebo");
        else
            ROS_ERROR("RP+ (TCP): failed to call service /gazebo/reset_world");
    }
    ROS_INFO("RP+ (TCP): finished resetting scenario");
}


void rexd_client::write_scenario() {
    std::string parser_path, source_path, domain_path, problem_path;
    node_handle->getParam("parser_path", parser_path);
    node_handle->getParam("source_path", source_path);
    node_handle->getParam("domain_path", domain_path);
    node_handle->getParam("problem_path", problem_path);
    std::string command = "python3 " + parser_path + " write_pddl " + source_path + " " + domain_path + " " + problem_path;
    system(command.c_str());
}

void rexd_client::problemTopic_callback(const std_msgs::String::ConstPtr& msg) {
    pddl_problem_string = msg->data;
    pddl_problem_string.erase(std::remove(pddl_problem_string.begin(), pddl_problem_string.end(), '\n'), pddl_problem_string.end()); // remove EOL
    ROS_INFO("RP+ (TCP): received problem topic");
    receive_problem = true;
}


void rexd_client::actionDispatchTopic_callback(const rosplan_dispatch_msgs::ActionDispatch& msg) {
    action_duration = msg.duration;
    ROS_INFO("RP+ (TCP): received duration of action = %f", action_duration);
    receive_action_dispatch = true;
}


void rexd_client::actionFeedbackTopic_callback(const rosplan_dispatch_msgs::ActionFeedback& msg) {
    action_feedback = msg.status;
    ROS_INFO("RP+ (TCP): received feedback for action = %s", action_feedback.c_str());
    receive_action_feedback = true;
}


std::string rexd_client::get_command() {
    std::string input_command;
    std::getline(*client_istream, input_command);
    return input_command;
}


void rexd_client::send_state() {
    receive_problem = false;
    // generate new problem.pddl so that current state can be parsed
    
    std_srvs::Empty problemSrv;
    ros::Rate r(1); // 1 hz
    while (ros::ok() and !generate_plan_client.call(problemSrv)) {
        ROS_INFO("RP+ (TCP): calling service /rosplan_problem_interface/problem_generation_server");
        r.sleep();
    }
    
    // now problemTopic_callback should be activated
    // ros::Time start_time = ros::Time::now();
    // ros::Duration timeout(3.0); // Timeout in seconds
    // while((ros::Time::now() - start_time) < timeout and !receive_problem) {
    while(ros::ok() and !receive_problem) {
        ROS_INFO("RP+ (TCP): waiting for state...");
        if (receive_problem) break;
        r.sleep();
        ros::spinOnce();
    }
    ROS_INFO("RP+ (TCP): writing state to %s", tmp_path.c_str());
    // *** write to tmp.txt ***
    std::string parser_path, source_path;
    node_handle->getParam("parser_path", parser_path);
    node_handle->getParam("source_path", source_path);
    std::string command = "python3 " + parser_path + " write_state_objects " + tmp_path + " " 
                            + source_path + simulator_ppddl_current_problem_file + " \"" + pddl_problem_string + "\"";

    system(command.c_str());
    *client_ostream << get_state() << std::endl << get_objects() << std::endl;
    *client_ostream << last_reward << std::endl;
}


void rexd_client::send_action_result(const std::string& result) const {
    ROS_INFO("RP+ (TCP): execution status = %s", result.c_str());
    *client_ostream << "action_finished_" << result << std::endl << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosplan_rexd_client");
    ros::NodeHandle nh("~");

    std::string host = "127.0.0.1";
    int portNo = 26550;
    nh.getParam("host", host);
    nh.getParam("port_no", portNo);
    
    // std::string problemTopic, actionDispatchTopic, actionFeedbackTopic;
    // nh.getParam("problem_topic", problemTopic);
    // nh.getParam("action_dispatch_topic", actionDispatchTopic);
    // nh.getParam("action_feedback_topic", actionFeedbackTopic);
    // std::cout << "Host: " << host << ", Port: " << portNo << ", Problem Topic: " << problemTopic
    //             << ", Action Dispatch Topic: " << actionDispatchTopic
    //             << ", Action Feedback Topic: " << actionFeedbackTopic << std::endl;

    std::string port = boost::lexical_cast<std::string>(portNo);
    rexd_client client(nh, host, port);

    bool continue_simulation = true;

    while (ros::ok() and continue_simulation) {
        std::string input_command = client.get_command();
        continue_simulation = client.parse_command(input_command);
        ros::spinOnce();
    }
    ROS_INFO("RP+ (TCP): terminate connection.");
    while (ros::ok())
        ros::Duration(10.0).sleep();    // because roslaunch sets respawn = true, we do an infinite loop to prevent node from respawning
    ros::shutdown();
}