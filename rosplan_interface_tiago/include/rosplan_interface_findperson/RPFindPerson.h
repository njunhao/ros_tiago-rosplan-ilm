#ifndef tiago_findperson
#define tiago_findperson

#include "rosplan_interface_search/RPSearch.h"
#include "pal_detection_msgs/Detections2d.h"
// #include <opencv2/highgui/highgui.hpp>    // only use if using imshow

#define PI 3.14159265358979323846  /* pi */


namespace KCL_rosplan {

	class RPFindPerson: public RPSearch<pal_detection_msgs::Detections2d>
	{

	protected:
		ros::Publisher pub_img;
		cv_bridge::CvImage cv_img;
		int counter;
		float scale_factor, camera_hfov;
		double dist_away;
		std::string fixed_frame;
		// std::string reserved_waypoint_instance;

		void detectionCallback(const pal_detection_msgs::Detections2d& msg);
		void initialize(ros::NodeHandle &nh);
		// std::string get_robot_at(const std::string& robot_instance) ;
		void end_process();	
		void clear_detections();	// if this is not virtual, methods of base class that calls clear_detections() will use the base class's method
		uint get_num_poi_detected();
		void handle_detections(cv_bridge::CvImageConstPtr cv_ptr);
		bool resize_window(uint id, int &x, int &y, int &width, int &height, float sf);
		bool run_simulated_action();
		void update_facts_in_kb(const std::string& robot_instance, const std::string& person_instance, const geometry_msgs::PoseStamped& pose);


	public:

		/* constructor */
		RPFindPerson(ros::NodeHandle &nh, std::string &actionserver);

	};

}
#endif