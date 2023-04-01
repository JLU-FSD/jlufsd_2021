#include "fsa_planning.h"

// Constructor
FSAPlanning::FSAPlanning()
{
    slam_map_topic_name_ = "/estimation/slam/map";
    slam_state_topic_name_ = "/estimation/slam/state";
    velocity_estimate_topic_name_ = "/estimation/velocity_estimation/velocity_estimate";
    max_speed_ = 3.0;
    slam_map_subscriber_ = nh_.subscribe(slam_map_topic_name_, 1, &FSAPlanning::slamMapCallback, this);
    slam_state_subscriber_ = nh_.subscribe(slam_state_topic_name_, 1, &FSAPlanning::slamStateCallback, this);
    velocity_estimate_subscriber_ = nh_.subscribe(velocity_estimate_topic_name_, 1, &FSAPlanning::velocityEstimateCallback, this);
    
    ROS_INFO("Constructing Planning");
}

void FSAPlanning::slamMapCallback(const auto_msgs::Map &map) {
    map_ = map;
    //桩桶摆放间隔大概为5m,去除多余的桩通
    const auto it_nearest_blue = std::min_element(map.blue_cone.begin(), map.blue_cone.end(),
                                            [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                        {
                                            const double da = std::hypot(state_.x - a.position.x, state_.y - a.position.y);
                                            const double db = std::hypot(state_.x - b.position.x, state_.y - b.position.y);

                                            return da < db;
                                        });
    std::vector<auto_msgs::Cone> cone_clean;
    std::vector<auto_msgs::Cone> cone_continuous;
    // ROS_INFO("old map_.blue_cone.size()   = %d",map_.blue_cone.size());
    // ROS_INFO("old map_.red_cone.size() = %d",map_.red_cone.size());
    cone_clean.clear();
    cone_continuous.clear();
    findNearestPoint(cone_clean, 
                    map_.blue_cone, 
                    state_.x, 
                    state_.y, 
                    state_.yaw,
                    0);
    map_.blue_cone = cone_clean;
    cone_clean.clear();
    cone_continuous.clear();
    findNearestPoint(cone_clean, 
                    map_.red_cone, 
                    state_.x, 
                    state_.y, 
                    state_.yaw,
                    1);
    map_.red_cone = cone_clean;
    // // ROS_INFO("new map_.blue_cone.size()   = %d",map_.blue_cone.size());
    // // ROS_INFO("new map_.red_cone.size() = %d",map_.red_cone.size());
    //将处理过的桩通输入给planning算法
    perceived_road_.left_boundry.path.clear();
    for(int i = 0; i < map_.blue_cone.size(); i++)
    {
        planning::PathPoint p;
        p.x = map_.blue_cone[i].position.x;
        p.y = map_.blue_cone[i].position.y;
        perceived_road_.left_boundry.path.push_back(p);
    }
    perceived_road_.right_boundry.path.clear();
    for(int i = 0; i < map_.red_cone.size(); i++)
    {
        planning::PathPoint p;
        p.x = map_.red_cone[i].position.x;
        p.y = map_.red_cone[i].position.y;
        perceived_road_.right_boundry.path.push_back(p);
    }
    // if(map_.cone_orange.size() > 0)
    // {//如果收到橙色信号说明处于发车位置,用于圈数lap的更新
    //     start_flag_ = true;
    // }
}

void FSAPlanning::findNearestPoint(std::vector<auto_msgs::Cone> &cone_clean, 
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

void FSAPlanning::slamStateCallback(const auto_msgs::CarState &state) {
    ROS_INFO("revieve p");
    state_.x = state.car_state.x;
    state_.y = state.car_state.y;
    state_.yaw = state.car_state.theta;
}

void FSAPlanning::velocityEstimateCallback(const auto_msgs::CarStateDt &velocity) {
    ROS_INFO("revieve v");
    state_.vx = velocity.car_state_dt.x;//纵向速度，即仪表盘速度
    state_.vy = velocity.car_state_dt.y;//侧向速度
    state_.r = velocity.car_state_dt.theta;//横摆角速度
}

void FSAPlanning::loopProc()
{
    if(!roadInfoCheckOK())return; 
    runAlgorithm();
    pubBountry();
    pubPath();
}

bool FSAPlanning::roadInfoCheckOK()
{
    if(perceived_road_.left_boundry.path.size() == 0 && perceived_road_.right_boundry.path.size() == 0)
    {
        ROS_ERROR("Signal of all the cones lost!");//桩桶信息全部丢失
        trajectory_.path.clear();
        return false;
    }
    if(perceived_road_.left_boundry.path.size() == 0)
    {
        ROS_ERROR("Signal of blue cone lost!");//蓝色桩桶信息丢失
        trajectory_.path.clear();
        return false;
    }
    if(perceived_road_.right_boundry.path.size() == 0)
    {
        ROS_ERROR("Signal of yellow cone lost!");//黄色桩桶信息丢失
        trajectory_.path.clear();
        return false;
    }
    return true;
}





