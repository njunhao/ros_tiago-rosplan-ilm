#ifndef tiago_talktoperson
#define tiago_talktoperson

#include "rosplan_interface_talk/RPTalk.h"
#include "pal_detection_msgs/Detections2d.h"
#include "pal_interaction_msgs/TtsAction.h"   // /media/alvin/HDD/Academics/PhD/Coding/ROS/tiago_ws/devel/share/pal_interaction_msgs/msg/TtsAction.msg


namespace KCL_rosplan {

	class RPTalkToPerson: public RPTalk<pal_interaction_msgs::TtsAction, pal_interaction_msgs::TtsGoal, pal_detection_msgs::Detections2d>
	{

	protected:
		bool detected_poi;
		std::string goals_file_path, speech_text;

		void initialize(ros::NodeHandle &nh);
		void detectionCallback(const pal_detection_msgs::Detections2d& msg);
		void start_process();
		void end_process(bool success);
		void get_action_msg(pal_interaction_msgs::TtsGoal& action_goal);
		bool facing_target();
		bool get_task_from_person(std::vector<rosplan_knowledge_msgs::KnowledgeItem>* items_ptr, std::string line);
		bool run_simulated_action();

	public:

		/* constructor */
		RPTalkToPerson(ros::NodeHandle &nh, std::string &actionserver);

	};

}
#endif
