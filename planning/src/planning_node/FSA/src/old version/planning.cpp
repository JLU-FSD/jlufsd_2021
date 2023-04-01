#include <ros/ros.h>
#include "planning.hpp"
#include <sstream>

#include <ctime>
#include <ratio>
#include <chrono>

// Constructor
Planning::Planning(ros::NodeHandle &nh){

    // if (!nh.param<double>("controller/speed/p", speed_p, 0.01)) {
    //     ROS_WARN_STREAM("Did not load controller/speed/p. Standard value is: " << 0.01);
    // }

}

//get函数
nav_msgs::Path Planning::visCenterLine() 
{ 
    return generatePathVisFromPath(local_road_.center_line); 
}

nav_msgs::Path Planning::visLeftBoundry() 
{ 
    return generatePathVisFromPath(local_road_.left_boundry);  
}

nav_msgs::Path Planning::visRightBoundry() 
{ 
    return generatePathVisFromPath(local_road_.right_boundry);
}

nav_msgs::Path Planning::visTrajectory() 
{ 
    return generatePathVisFromPath(trajectory_);
}


//set函数
void Planning::setMaxSpeed(double &max_speed) {
    max_speed_ = max_speed;
}

bool Planning::roadInfoCheckOK()
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

void Planning::lapRecord()
{
    if(start_state_recorded_ == false)
    {
        start_state_ = state_;
        start_state_recorded_ = true;
    }

    if(start_flag_ && planning::distance(start_state_, state_) < 3 && !in_start_state_)
    {//通过双重是否检测到橙色桩通，以及是否位于发车位置判定是否，尤其是第三个条件，必须从不在起点附近的地方再次接近起点
        lap++;
        global_road_.left_boundry.showInCsv("/home/auto/fsd/src/planning/data/left_boundry.csv");
        global_road_.right_boundry.showInCsv("/home/auto/fsd/src/planning/data/right_boundry.csv");
        in_start_state_ = true;
    }

    if(in_start_state_)
    {
        if(!start_flag_ || planning::distance(start_state_, state_) >= 3)
        {//消除处于发车位置状态的条件
            in_start_state_ = false;
        }
    }
}

void Planning::runAlgorithm() {
    /////////////////////////
    // planning::Path path;
    // planning::readFromCsv(path, "/home/auto/fsd/src/planning/data/left_origin_save1.csv");
    // spline_.smoothProc(path).showInCsv("/home/auto/fsd/src/planning/data/result.csv");
     
    //////////////////////////
    if(!roadInfoCheckOK())return;   
    lapRecord(); 

    if(lap == 0)
    {
        ROS_WARN("lap---0!Go!");
        expandGlobalMap();
        // createLocalMap();
        ROS_WARN("lap---0!Go! = %d",global_road_.left_boundry.path.size());
        ROS_WARN("lap---0!Go! = %d",global_road_.right_boundry.path.size());
        local_road_ = perceived_road_;
        createLocalDenseMap();
        createCenterLine();
        // global_road_.center_line.showInCsv("/home/auto/fsd/src/planning/data/global_road_.csv");
    }

    if(lap == 1)
    {
        createGlobalDenseMap();
        ROS_WARN("check 0");
        // min_cur_optimizer_.process(global_road_.center_line);
        ROS_WARN("lap---1!Go!");
    }

    if(lap == 2)
    {
        ROS_WARN("lap---2!Go!");
    }
    

    
    // createRoadWidth(dense_map_, center_line_);
    // center_line_.showInCsv("/home/auto/fsd/src/planning/data/centerline.csv");
    // ROS_INFO("Constructing Handle------------------------------");
    // createMaxSpeed(center_line_);
    
    createTrajectory();
}

void Planning::expandGlobalMap() {
    //将感知到的点按照顺序添加到全局地图中去,注意保证新点与最后一个点的距离与角度约束
    bool in_reserve_road_flag;
    // planning::PathPointPtr left_final_global_point = global_road_.left_boundry.path.end() - 1;
    // planning::PathPointPtr right_final_global_point = global_road_.right_boundry.path.end() - 1;
    //左侧桩桶
    for(auto &left_point: perceived_road_.left_boundry.path)
    { 
        if( !planning::isPointInPath(left_point, global_road_.left_boundry) )
        {//如果不在全局地图中
            in_reserve_road_flag = false;

            for(auto point_ptr = reserve_road_.left_boundry.path.begin();
                     point_ptr != reserve_road_.left_boundry.path.end(); 
                     point_ptr++)
            {
                if(planning::distance(*point_ptr, left_point) < 0.5)
                {//如果在预备地图中，增加预备的次数
                    point_ptr->s += 1.0;//用s作为累计的次数
                    if(point_ptr->s >= 5)
                    {//如果达到预备次数阈值，加入全局地图，并清除出预备地图
                        global_road_.left_boundry.path.push_back(left_point);
                        reserve_road_.left_boundry.path.erase(point_ptr);
                    }
                    in_reserve_road_flag = true;
                    break;
                }
            }
            if(in_reserve_road_flag == false)
            {//如果不在预备地图中，加入预备地图
                left_point.s = 0;
                reserve_road_.left_boundry.path.push_back(left_point);
            }
        }
    }
    //右侧桩桶
    for(auto &right_point: perceived_road_.right_boundry.path)
    {
        if( !planning::isPointInPath(right_point, global_road_.right_boundry) )
        {//如果不在全局地图中
            in_reserve_road_flag = false;

            for(auto point_ptr = reserve_road_.right_boundry.path.begin();
                     point_ptr != reserve_road_.right_boundry.path.end(); 
                     point_ptr++)
            {
                if(planning::distance(*point_ptr, right_point) < 0.5 )
                {//如果在预备地图中，增加预备的次数
                    point_ptr->s += 1.0;
                    if(point_ptr->s >= 10)
                    {//如果达到预备次数阈值，加入全局地图，并清除出预备地图
                        global_road_.right_boundry.path.push_back( right_point );
                        reserve_road_.right_boundry.path.erase( point_ptr );
                    }
                    in_reserve_road_flag = true;
                    break;
                }
            }
            if(in_reserve_road_flag == false)
            {//如果不在预备地图中，加入预备地图
                right_point.s = 0;
                reserve_road_.right_boundry.path.push_back(right_point);
            }
        }
    }
    // global_road_.right_boundry.showInCsv("/home/auto/fsd/src/planning/data/global_right.csv");
}

void Planning::createLocalMap() {
    // global_road_
    // local_road_
    //左侧桩桶
    const auto it_nearest_left = std::min_element(global_road_.left_boundry.path.begin(), global_road_.left_boundry.path.end(),
                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.x, state_.y - a.y);
                                        const double db = std::hypot(state_.x - b.x, state_.y - b.y);
                                        return da < db;
                                    });
    local_road_.left_boundry.path.clear();
    for(auto it = it_nearest_left; it != global_road_.left_boundry.path.end(); it++)
    {//从距离车辆最近的桩桶开始到最后一个桩桶，第一圈可以用这个方法
        local_road_.left_boundry.path.push_back(*it);
    }

    //右侧桩桶
    const auto it_nearest_right = std::min_element(global_road_.right_boundry.path.begin(), global_road_.right_boundry.path.end(),
                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.x, state_.y - a.y);
                                        const double db = std::hypot(state_.x - b.x, state_.y - b.y);
                                        return da < db;
                                    });
    local_road_.right_boundry.path.clear();
    for(auto it = it_nearest_right; it != global_road_.right_boundry.path.end(); it++)
    {//从距离车辆最近的桩桶开始到最后一个桩桶，第一圈可以用这个方法
        local_road_.right_boundry.path.push_back(*it);
    }
}

void Planning::createLocalDenseMap() {
    // local_road_.left_boundry = spline_.smoothProc(local_road_.left_boundry);
    // local_road_.right_boundry = spline_.smoothProc(local_road_.right_boundry);

    // local_road_.left_boundry = pure_pursuit_smoother_.process(local_road_.left_boundry);
    // local_road_.right_boundry = pure_pursuit_smoother_.process(local_road_.right_boundry);

    // local_road_.left_boundry = stanley_smoother_.process(local_road_.left_boundry);
    // local_road_.right_boundry = stanley_smoother_.process(local_road_.right_boundry);

    // local_road_.left_boundry = lqr_smoother_.process(local_road_.left_boundry);
    // local_road_.right_boundry = lqr_smoother_.process(local_road_.right_boundry);

    // local_road_.left_boundry = rotate_3order_smoother_.process(local_road_.left_boundry);
    // local_road_.right_boundry = rotate_3order_smoother_.process(local_road_.right_boundry);

    local_road_.left_boundry = iterative_polynomial_smoother_.process(local_road_.left_boundry);
    local_road_.right_boundry = iterative_polynomial_smoother_.process(local_road_.right_boundry);
}

void Planning::createGlobalDenseMap() {
    global_road_.left_boundry = iterative_polynomial_smoother_.process(local_road_.left_boundry);
    global_road_.right_boundry = iterative_polynomial_smoother_.process(local_road_.right_boundry);
}


void Planning::createCenterLine() {
    if (local_road_.left_boundry.path.size() == 0 || local_road_.right_boundry.path.size() == 0)
    {
        return;
    }
    
    local_road_.center_line.path.clear();
    for (const auto &right_point: local_road_.right_boundry.path) {
        //找出距离当前左侧车道点最近的点
        const auto it_left = std::min_element(local_road_.left_boundry.path.begin(), local_road_.left_boundry.path.end(),
                                                [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(right_point.x - a.x, right_point.y - a.y);
                                                    const double db = std::hypot(right_point.x - b.x, right_point.y - b.y);

                                                    return da < db;
                                                });
        if(std::hypot(right_point.x - it_left->x, right_point.y - it_left->y) < 3 *1.8
        &&std::hypot(right_point.x - it_left->x, right_point.y - it_left->y) > 3*0.5)
        {
            planning::PathPoint p;
            p.x = static_cast<float>((right_point.x + it_left->x) / 2.0);
            p.y = static_cast<float>((right_point.y + it_left->y) / 2.0);
            p.cur = static_cast<float>((right_point.cur + it_left->cur) / 2.0);
            local_road_.center_line.path.push_back(p);
        }
    }
    local_road_.center_line.calculateS();
    local_road_.center_line.calculateYaw();
    // ROS_INFO("path = %d", road_.center_line.path.size());
    // ROS_INFO("yellow = %d", road_.right_boundry.path.size());
    // ROS_INFO("blue = %d", road_.left_boundry.path.size());
}

// void Planning::createRoadWidth(const fsd_common_msgs::Map &map, planning::Path &path) {
//     
// }

// void Planning::createMaxSpeed(planning::Path &path) {
//     //斯坦福ddl的最优速度计算
//     // max_speed_
// }

void Planning::createTrajectory()
{
    
    // state_;
    // velocity_;

    // center_line_;
    // trajectory_;
    trajectory_ = local_road_.center_line;

    // if(in_start_state_)
    // {
    //     if(std::hypot(start_state_.x - state_.x, start_state_.y - state_.y) > 10)
    //     {
    //         in_start_state_ = false;
    //     }
    //     ROS_WARN("Start State---3!2!1!Go!");
    //     trajectory_.path.clear();
    //     for(double distance = 0; distance < 10; distance++)
    //     {
    //         planning::PathPoint p; 
    //         p.x = state_.x + distance * cos(start_state_.yaw);
    //         p.y = state_.y + distance * sin(start_state_.yaw);
    //         p.yaw = state_.yaw;
            
    //         trajectory_.path.push_back(p);
    //     }
    // }
}