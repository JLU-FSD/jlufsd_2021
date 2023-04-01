#include "planning_core.h"

Planning::Planning()
{
    path_topic_name_ = "/planning/path";
    left_boundry_topic_name_ = "/planning/left_boundry";
    right_boundry_topic_name_ = "/planning/right_boundry";
    left_boundry_publisher_ = nh_.advertise<nav_msgs::Path>(left_boundry_topic_name_, 1, true);
    right_boundry_publisher_ = nh_.advertise<nav_msgs::Path>(right_boundry_topic_name_, 1, true);
    path_publisher_ = nh_.advertise<nav_msgs::Path>(path_topic_name_, 1, true);
    setLoopRate(10);
}

//PUB函数
void Planning::pubBountry()
{
    left_boundry_for_vis_ = visLeftBoundry();
    left_boundry_for_vis_.header.frame_id = "map";
    left_boundry_for_vis_.header.stamp    = ros::Time::now();
    left_boundry_publisher_.publish(left_boundry_for_vis_);
    
    right_boundry_for_vis_ = visRightBoundry();
    right_boundry_for_vis_.header.frame_id = "map";
    right_boundry_for_vis_.header.stamp    = ros::Time::now();
    right_boundry_publisher_.publish(right_boundry_for_vis_);
}

void Planning::pubPath() 
{
    path_for_vis_ = visPath();
    path_for_vis_.header.frame_id = "map";
    path_for_vis_.header.stamp = ros::Time::now();
    path_publisher_.publish(path_for_vis_);
}

//VIS函数
nav_msgs::Path Planning::visLeftBoundry() 
{ 
    return  generatePathVisFromPath(left_boundry_);  
}

nav_msgs::Path Planning::visRightBoundry() 
{ 
    return generatePathVisFromPath(right_boundry_);
}

nav_msgs::Path Planning::visPath() 
{ 
    return generatePathVisFromPath(trajectory_);
}

