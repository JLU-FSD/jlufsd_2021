#pragma once
#include "ros/ros.h"
#include "planning_common.h"

class Planning
{
public:
	// Constructor
    Planning();

    virtual void loopProc() = 0;//虚方法=0表示子类必须实现
	void setLoopRate(double _loop_rate){loop_rate_ = _loop_rate;};
	double getLoopRate() const {return loop_rate_;};
	void pubBountry();
    void pubPath();
	nav_msgs::Path visLeftBoundry();
    nav_msgs::Path visRightBoundry();
    nav_msgs::Path visPath();

	ros::NodeHandle 			nh_;
	planning::Vehicle           state_;
	planning::Path              trajectory_;
	planning::Path				left_boundry_;
	planning::Path				right_boundry_;
	ros::Publisher 				planning_publisher_;//need to be conducted in the 
private:
	double 						loop_rate_ = 20;
    std::string                 left_boundry_topic_name_;
    std::string                 right_boundry_topic_name_;
	std::string                 path_topic_name_;
	ros::Publisher 				left_boundry_publisher_;
	ros::Publisher 				right_boundry_publisher_;
	ros::Publisher 				path_publisher_;//could be set color in future
	nav_msgs::Path              left_boundry_for_vis_;
    nav_msgs::Path              right_boundry_for_vis_;
	nav_msgs::Path              path_for_vis_;
};