#pragma once
#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "auto_msgs/Map.h"
#include "auto_msgs/CarState.h"
#include "auto_msgs/CarStateDt.h"
#include "planning_core.h"

class FSAPlanning : public Planning
{
public:
    // Constructor
    explicit FSAPlanning();
    
    // Methods
    bool initalize(){};
    void velocityEstimateCallback(const auto_msgs::CarStateDt &velocity);
    void slamMapCallback(const auto_msgs::Map &map);
    void slamStateCallback(const auto_msgs::CarState &state);
    void loopProc();
    bool roadInfoCheckOK();
    virtual void runAlgorithm() = 0;//虚方法=0表示子类必须实现
    virtual void switchStage() = 0;

    planning::Road              perceived_road_;
    double                      max_speed_;
    std::string                 stage_;//record the racing stage.(state machine methord)
    auto_msgs::Map              map_;
private:
    ros::Subscriber             slam_map_subscriber_;
    ros::Subscriber             slam_state_subscriber_;
    ros::Subscriber             velocity_estimate_subscriber_;
    std::string                 velocity_estimate_topic_name_;
    std::string                 slam_map_topic_name_;
    std::string                 slam_state_topic_name_;
    
    //最后一项0代表向左，1代表向右
    void findNearestPoint(std::vector<auto_msgs::Cone> &cone_clean, 
                                            std::vector<auto_msgs::Cone> &cone_set, 
                                            double & x, double & y, double & yaw, 
                                            int direction);
};