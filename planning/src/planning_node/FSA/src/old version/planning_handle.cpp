#include <ros/ros.h>
#include "planning_handle.hpp"

#include <ctime>

#include <ratio>
#include <chrono>

// ROS Msgs
// #include "geometry_msgs/PolygonStamped.h"

// Constructor
PlanningHandle::PlanningHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    planning_(nodeHandle) {
    ROS_INFO("Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
    planning_.setMaxSpeed(max_speed_);
}

void PlanningHandle::run() 
{
    planning_.runAlgorithm();
    sendBountry();
    sendCenterLine();
    sendTrajectory();
    sendSimControl();
    sendConePointCloud();
}
// Sender Methods
void PlanningHandle::sendCenterLine() 
{
    center_line_ = planning_.visCenterLine();
    center_line_.header.frame_id = "map";
    center_line_.header.stamp    = ros::Time::now();
    centerLinePublisher_.publish(center_line_);
}

void PlanningHandle::sendTrajectory() 
{
    trajectory_ = planning_.visTrajectory();
    trajectory_.header.frame_id = "map";
    trajectory_.header.stamp    = ros::Time::now();
    trajectoryPublisher_.publish(trajectory_);
}

void PlanningHandle::sendSimControl() 
{
    trajectory_ = planning_.visTrajectory();
    auto_msgs::ControlCommand control_command;

    if (trajectory_.poses.empty()) {
        ROS_WARN("NO PATH 1 ");
        control_command.throttle.data       = static_cast<float>(0.5);
        control_command.steering_angle.data = 0.0;
        return;
    }

    const auto it_now = std::min_element(trajectory_.poses.begin(), trajectory_.poses.end(),
                                        [&](const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) 
                                        {
                                            const double da = std::hypot(planning_.state_.x - a.pose.position.x,
                                                                        planning_.state_.y - a.pose.position.y);
                                            const double db = std::hypot(planning_.state_.x - b.pose.position.x,
                                                                        planning_.state_.y - b.pose.position.y);
                                            return da < db;
                                        });
    const auto             i_now          = std::distance(trajectory_.poses.begin(), it_now);//最近点是第几个点
    const auto             size           = trajectory_.poses.size();
    // const auto             i_next         = (i_now + 10) % size;//10个点之后的点确定为预瞄点
    int i_next = 0;
    for(int i = i_now; i < trajectory_.poses.size(); i++)
    {
        if(std::hypot(trajectory_.poses[i].pose.position.y - planning_.state_.y, 
                        trajectory_.poses[i].pose.position.x - planning_.state_.x) > 5)
        {
            i_next = i;
            break;
        }
        i_next = i;
    }

    float steering_p = 1.2;
    float speed_p = 0.015;
    float max_speed_ = 4;

    geometry_msgs::PoseStamped next_point = trajectory_.poses[i_next];

    { // Steering Control
        const double beta_est = control_command.steering_angle.data * 0.5;
        const double eta      = std::atan2(next_point.pose.position.y - planning_.state_.y,    
                                            next_point.pose.position.x - planning_.state_.x) 
                                            - (planning_.state_.yaw + beta_est);
        const double length   = std::hypot(next_point.pose.position.y - planning_.state_.y, 
                                            next_point.pose.position.x - planning_.state_.x);
        control_command.steering_angle.data = static_cast<float>(steering_p * std::atan(2.0 / length * std::sin(eta)));
    }

    { // Speed Controller
        const double vel = std::hypot(planning_.state_.vx, planning_.state_.vy);
        control_command.throttle.data = static_cast<float>(speed_p * (max_speed_ - vel));
    }
    //如果信号丢失则减速
    if(trajectory_.poses.size() == 0)
    {
        ROS_WARN("NO PATH");
        // control_command.throttle.data = -0.05;
        control_command.steering_angle.data = 0;
    }
    double theta = std::atan2(next_point.pose.position.y - planning_.state_.y,    
                                next_point.pose.position.x - planning_.state_.x);
    theta = std::abs(planning::angleDifference( theta, planning_.state_.yaw ));

    //如果跟随点落后于本车位置
    if(theta > M_PI_2)
    {
        ROS_WARN("Point behind vehicle, theta = %f", theta);
        // control_command.throttle.data = 0.05;
        control_command.steering_angle.data = 0;
    }
    //如果跟随点距离本车过近
    double distance = std::hypot(next_point.pose.position.y - planning_.state_.y, 
                                next_point.pose.position.x - planning_.state_.x);
    if(distance < 0.6)
    {
        ROS_WARN("TOO CLOSE");
        // control_command.throttle.data = 0.05;
        control_command.steering_angle.data = 0;
    }
    // ROS_WARN("\nsteering_angle = %f  ;\nthrottle = %f", control_command.steering_angle.data,control_command.throttle.data);
    control_command.header.frame_id = "map";
    control_command.header.stamp    = ros::Time::now();
    // controlPublisher_.publish(control_command);

    follow_point_.header.frame_id = "map";
    follow_point_.header.stamp    = ros::Time::now();
    follow_point_.point.x = trajectory_.poses[i_next].pose.position.x;
    follow_point_.point.y = trajectory_.poses[i_next].pose.position.y;
    follow_point_.point.z = 0;
    followPointPublisher_.publish(follow_point_);
}

void PlanningHandle::sendConePointCloud() 
{
    pcl::PointCloud<pcl::PointXYZ> yellow_point_cloud;
    sensor_msgs::PointCloud2 yellow_point_cloud_msg;
    yellow_point_cloud.clear();
    for(auto &cone : map_.cone_yellow)
    {   
        pcl::PointXYZ point;
        point.x = cone.position.x;
        point.y = cone.position.y;
        point.z = 0;
        yellow_point_cloud.push_back(point);
    }
    pcl::toROSMsg(yellow_point_cloud, yellow_point_cloud_msg);
    yellow_point_cloud_msg.header.frame_id = "map";
    yellow_point_cloud_msg.header.stamp    = ros::Time::now();
    yellowConePublisher_.publish(yellow_point_cloud_msg);
    // ROS_INFO("yellow_point_cloud for planning to visualize sended. There are");

    pcl::PointCloud<pcl::PointXYZ> blue_point_cloud;
    sensor_msgs::PointCloud2 blue_point_cloud_msg;
    blue_point_cloud.clear();
    for(auto &cone : map_.cone_blue)
    {   
        pcl::PointXYZ point;
        point.x = cone.position.x;
        point.y = cone.position.y;
        point.z = 0; 
        blue_point_cloud.push_back(point);
    }
    pcl::toROSMsg(blue_point_cloud, blue_point_cloud_msg);
    blue_point_cloud_msg.header.frame_id = "map";
    blue_point_cloud_msg.header.stamp    = ros::Time::now();
    blueConePublisher_.publish(blue_point_cloud_msg);
    // ROS_INFO("blue_point_cloud_msg for planning to visualize sended. There are");
}

void PlanningHandle::sendBountry()
{
    left_boundry_ = planning_.visLeftBoundry();
    left_boundry_.header.frame_id = "map";
    left_boundry_.header.stamp    = ros::Time::now();
    leftBoundryPublisher_.publish(left_boundry_);
    
    right_boundry_ = planning_.visRightBoundry();
    right_boundry_.header.frame_id = "map";
    right_boundry_.header.stamp    = ros::Time::now();
    rightBoundryPublisher_.publish(right_boundry_);
}

// Methods
void PlanningHandle::loadParameters() {
    ROS_INFO("loading handle parameters");
    if (!nodeHandle_.param("max_speed", max_speed_, 3.0)) 
    {
        ROS_WARN_STREAM("Did not load max_speed. Standard value is: " << max_speed_);
    }

    if (!nodeHandle_.param<std::string>("slam_map_topic_name", slam_map_topic_name_, "/estimation/slam/map")) 
    {
        ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << slam_map_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("slam_state_topic_name", slam_state_topic_name_, "/estimation/slam/state")) 
    {
        ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("velocity_estimate_topic_name", velocity_estimate_topic_name_, "/estimation/velocity_estimation/velocity_estimate")) 
    {
        ROS_WARN_STREAM("Did not load velocity_estimate_topic_name. Standard value is: " << velocity_estimate_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("trajectory_topic_name", trajectory_topic_name_, "/planning/trajectory"))
    {
        ROS_WARN_STREAM("Did not load trajectory_topic_name. Standard value is: " << trajectory_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("center_line_topic_name", center_line_topic_name_, "/planning/center_line")) 
    {
        ROS_WARN_STREAM("Did not load center_line_topic_name. Standard value is: " << center_line_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("control_topic_name", control_topic_name_, "/control/pure_pursuit/control_command")) 
    {
        ROS_WARN_STREAM("Did not load control_topic_name. Standard value is: " << control_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("blue_point_cloud_name", blue_point_cloud_name_, "/planning/blue_point_cloud")) 
    {
        ROS_WARN_STREAM("Did not load blue_point_cloud_name. Standard value is: " << blue_point_cloud_name_);
    }

    if (!nodeHandle_.param<std::string>("yellow_point_cloud_name", yellow_point_cloud_name_, "/planning/yellow_point_cloud")) 
    {
        ROS_WARN_STREAM("Did not load yellow_point_cloud_name. Standard value is: " << yellow_point_cloud_name_);
    }

    if (!nodeHandle_.param<std::string>("left_boundry_topic_name", left_boundry_topic_name_, "/planning/left_boundry")) 
    {
        ROS_WARN_STREAM("Did not load blue_point_cloud_name. Standard value is: " << left_boundry_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("right_boundry_topic_name", right_boundry_topic_name_, "/planning/right_boundry")) 
    {
        ROS_WARN_STREAM("Did not load yellow_point_cloud_name. Standard value is: " << right_boundry_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("follow_point_topic_name", follow_point_topic_name_, "/planning/follow_point")) 
    {
        ROS_WARN_STREAM("Did not load yellow_point_cloud_name. Standard value is: " << follow_point_topic_name_);
    }

    if (!nodeHandle_.param("node_rate", node_rate_, 10)) 
    {
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    }
}

void PlanningHandle::subscribeToTopics() 
{
    ROS_INFO("subscribe to topics");
    slamMapSubscriber_          =
        nodeHandle_.subscribe(slam_map_topic_name_, 1, &PlanningHandle::slamMapCallback, this);
    slamStateSubscriber_        =
        nodeHandle_.subscribe(slam_state_topic_name_, 1, &PlanningHandle::slamStateCallback, this);
    velocityEstimateSubscriber_ =
        nodeHandle_.subscribe("/estimation/velocity_estimation/velocity_estimation", 1, &PlanningHandle::velocityEstimateCallback, this);
}

void PlanningHandle::publishToTopics() 
{
    ROS_INFO("publish to topics");
    trajectoryPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(trajectory_topic_name_, 1);
    centerLinePublisher_ = nodeHandle_.advertise<nav_msgs::Path>(center_line_topic_name_, 1, true);
    // controlPublisher_ = nodeHandle_.advertise<auto_msgs::ControlCommand>(control_topic_name_, 1, true);
    followPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>(follow_point_topic_name_, 1, true);

    yellowConePublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(yellow_point_cloud_name_, 1, true);
    blueConePublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(blue_point_cloud_name_, 1, true);

    leftBoundryPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(left_boundry_topic_name_, 1, true);
    rightBoundryPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(right_boundry_topic_name_, 1, true);
}




void PlanningHandle::findNearestPoint(std::vector<auto_msgs::Cone> &cone_clean, 
                                        std::vector<auto_msgs::Cone> &cone_set, 
                                        double & x, double & y, double & yaw, 
                                        int direction)
{
    if(cone_set.begin() == cone_set.end()-1)
    {
        return;
    }
    if(cone_set.size() > 0)
    {
        auto it_nearest = cone_set.begin();

        
        const auto it = std::min_element(cone_set.begin(), cone_set.end(),
                                        [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                        {
                                            const double da = std::hypot(x - a.position.x, y - a.position.y);
                                            const double db = std::hypot(x - b.position.x, y - b.position.y);

                                            return da < db;
                                        });
        it_nearest = it;


        double nearest_distance = std::hypot(x - it_nearest->position.x, y - it_nearest->position.y);
        if(cone_clean.size() == 0)
        {
            cone_clean.push_back(*it_nearest);
            double x = it_nearest->position.x;
            double y = it_nearest->position.y;
            cone_set.erase(it_nearest);
            findNearestPoint(cone_clean, cone_set, x, y, yaw, direction);
        }
        else
        {
            if( nearest_distance < 5 * 1.5 && nearest_distance > 5 * 0.2)//这里的5是标准桩通距离,1.8是容忍的偏差
            {    
                double yaw_new = atan2(it_nearest->position.y - y,it_nearest->position.x - x);
                //判断角度变化，认为角度变化超过 pi/4，则属于错误识别
                if(cone_clean.size() == 1)
                {//在只有一个点的时候无法形成初始角度
                    cone_clean.push_back(*it_nearest);
                    double x = it_nearest->position.x;
                    double y = it_nearest->position.y;
                    cone_set.erase(it_nearest);
                    findNearestPoint(cone_clean, cone_set, x, y, yaw_new, direction);
                }
                else
                {
                    // ROS_WARN("direction = %d",direction);
                    // ROS_WARN("std::abs(yaw - theta) = %f",std::abs(yaw - theta));
                    if( std::abs(planning::angleDifference(yaw, yaw_new)) < 60.0 / 180.0 * M_PI)
                    {
                        cone_clean.push_back(*it_nearest);
                        double x = it_nearest->position.x;
                        double y = it_nearest->position.y;
                        cone_set.erase(it_nearest);
                        findNearestPoint(cone_clean, cone_set, x, y, yaw_new, direction);
                    }
                    else
                    {
                        // ROS_ERROR("******angle******");
                        cone_set.erase(it_nearest);
                        findNearestPoint(cone_clean, cone_set, x, y, yaw, direction);
                    }               
                }
            }
            else
            {
                // ROS_ERROR("******distance******");
                cone_set.erase(it_nearest);
                findNearestPoint(cone_clean, cone_set, x, y, yaw, direction);
            }
        }
    }
}

void PlanningHandle::slamMapCallback(const auto_msgs::Map &map) {
    map_ = map;
    //桩桶摆放间隔大概为5m,去除多余的桩通
    const auto it_nearest_blue = std::min_element(map.cone_blue.begin(), map.cone_blue.end(),
                                            [&](const fsd_common_msgs::Cone &a, const fsd_common_msgs::Cone &b) 
                                        {
                                            const double da = std::hypot(planning_.state_.x - a.position.x, planning_.state_.y - a.position.y);
                                            const double db = std::hypot(planning_.state_.x - b.position.x, planning_.state_.y - b.position.y);

                                            return da < db;
                                        });

    std::vector<fsd_common_msgs::Cone> cone_clean;
    std::vector<fsd_common_msgs::Cone> cone_continuous;

    // ROS_INFO("old map_.cone_blue.size()   = %d",map_.cone_blue.size());
    // ROS_INFO("old map_.cone_yellow.size() = %d",map_.cone_yellow.size());

    cone_clean.clear();
    cone_continuous.clear();
    findNearestPoint(cone_clean, 
                    map_.cone_blue, 
                    planning_.state_.x, 
                    planning_.state_.y, 
                    planning_.state_.yaw,
                    0);
    map_.cone_blue = cone_clean;

    cone_clean.clear();
    cone_continuous.clear();
    findNearestPoint(cone_clean, 
                    map_.cone_yellow, 
                    planning_.state_.x, 
                    planning_.state_.y, 
                    planning_.state_.yaw,
                    1);
    map_.cone_yellow = cone_clean;

    // ROS_INFO("new map_.cone_blue.size()   = %d",map_.cone_blue.size());
    // ROS_INFO("new map_.cone_yellow.size() = %d",map_.cone_yellow.size());

    //将处理过的桩通输入给planning算法
    planning_.perceived_road_.left_boundry.path.clear();
    for(int i = 0; i < map_.cone_blue.size(); i++)
    {
        planning::PathPoint p;
        p.x = map_.cone_blue[i].position.x;
        p.y = map_.cone_blue[i].position.y;
        planning_.perceived_road_.left_boundry.path.push_back(p);
    }
    planning_.perceived_road_.right_boundry.path.clear();
    
    for(int i = 0; i < map_.cone_yellow.size(); i++)
    {
        planning::PathPoint p;
        p.x = map_.cone_yellow[i].position.x;
        p.y = map_.cone_yellow[i].position.y;
        planning_.perceived_road_.right_boundry.path.push_back(p);
    }

    
    if(map_.cone_orange.size() > 0)
    {//如果收到橙色信号说明处于发车位置,用于圈数lap的更新
        planning_.start_flag_ = true;
    }

}

void PlanningHandle::slamStateCallback(const auto_msgs::CarState &state) {
    ROS_INFO("revieve p");
    planning_.state_.x = state.car_state.x;
    planning_.state_.y = state.car_state.y;
    planning_.state_.yaw = state.car_state.theta;
}

void PlanningHandle::velocityEstimateCallback(const auto_msgs::CarStateDt &velocity) {
    ROS_INFO("revieve v");
    planning_.state_.vx = velocity.car_state_dt.x;//纵向速度，即仪表盘速度
    planning_.state_.vy = velocity.car_state_dt.y;//侧向速度
    planning_.state_.r = velocity.car_state_dt.theta;//横摆角速度
    ROS_INFO("planning_.state_.vx = velocity.car_state_dt.x = %f", planning_.state_.vx);
}