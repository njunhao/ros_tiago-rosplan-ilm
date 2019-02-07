#ifndef ext_search
#define ext_search

#include "rosplan_interface_explore/RPExplore.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


namespace KCL_rosplan {

	template <class T>
	class RPSearch: public RPExplore
	{

	protected:
		ros::Subscriber sub_detector, sub_image;
		bool searching, stop_exploring, action_succeeded;
		std::vector<geometry_msgs::Pose> poi_arr;
		T detections;

		virtual void detectionCallback(const T& msg) = 0;
		virtual void end_process() = 0;
		virtual void clear_detections() = 0;
		virtual uint get_num_poi_detected() = 0;
		virtual void handle_detections(cv_bridge::CvImageConstPtr cv_ptr) = 0;


		void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
			if (!searching or !detected_poi()) return;

		    try {
		    	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
		    	handle_detections(cv_ptr);
		    } catch (const cv_bridge::Exception& e) {
		        ROS_ERROR("RP+: (%s) cv_bridge exception: %s", params.name.c_str(), e.what());
		    }
		}
		

		virtual void start_process() {
			nav_msgs::Path empty_path;
			exploration_path = empty_path;
			searching = true;
			stop_exploring = false;
			action_succeeded = false;
			clear_detections();
		}


		virtual void initialize(ros::NodeHandle &nh) {
			std::string topic;
			nh.param("detector_topic", topic, std::string("/detections"));
	    	sub_detector = nh.subscribe(topic, 1, &RPSearch::detectionCallback, this);
	    	nh.param("depth_image_topic", topic, std::string("/image_raw"));
	    	sub_image = nh.subscribe(topic, 1, &RPSearch::depthImageCallback, this);
		}


		virtual bool detected_poi() { return get_num_poi_detected() > 0; }


		virtual bool continue_exploring() { return !stop_exploring; }

		
		bool is_successful() { return action_succeeded; }
		

	public:
		/* constructor */
		RPSearch(ros::NodeHandle &nh, std::string &actionserver)
	 		: KCL_rosplan::RPExplore(nh, actionserver) {
	 		initialize(nh);
		}
	};

}
#endif