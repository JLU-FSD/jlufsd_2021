#include "fsa_racetrack_planning.h"

bool FSARacetrackPlanning::initalize()
{
    no_yellow_high_cone_count_ = 0;
    lap_num_ = 1;
    stage_ = "Lap " + std::to_string(lap_num_);
    longitudinal_planner_.setMaxV(max_speed_);
}

void FSARacetrackPlanning::switchStage()
{
    if(is_near_start_ == true)
    {
        if(map_.yellow_high_cone.size() == 0)
        {
            if(++no_yellow_high_cone_count_ > 5 * getLoopRate())
            {//5s
                is_near_start_ = false;
            }
        }
    }
    else
    {
        no_yellow_high_cone_count_ = 0;
        for(auto cone : map_.yellow_high_cone)
        {
            if((pow((cone.position.x - state_.x), 2) + pow((cone.position.y - state_.y), 2)) < 5 * 5)
            {//如果黄色高桩通出现在5m之内
                stage_ = "Lap " + std::to_string(++lap_num_);
                is_near_start_ = true;
                return;
            }
        }
    }
}

void FSARacetrackPlanning::runAlgorithm()
{
    switchStage();
    if(stage_ == "Lap 1")
    {
        //生成稠密的局部地图
        local_road_.left_boundry = lateral_planner_.process(perceived_road_.left_boundry);
        local_road_.right_boundry = lateral_planner_.process(perceived_road_.right_boundry);
        //生成道路中心线
        local_road_.generateCenterLine();
        trajectory_ = local_road_.center_line;
        //补充道路信息
        trajectory_.calculateS();
        trajectory_.calculateYaw();
        //纵向speed规划
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.process(&trajectory_);
    }

    if((stage_ == "Lap 2" || stage_ == "Lap 3") && (!is_adopting_global_method_))
    {
        //生成稠密的局部地图
        local_road_.left_boundry = lateral_planner_.process(perceived_road_.left_boundry);
        local_road_.right_boundry = lateral_planner_.process(perceived_road_.right_boundry);
        //生成道路中心线
        local_road_.generateCenterLine();
        trajectory_ = local_road_.center_line;
        //补充道路信息
        trajectory_.calculateS();
        trajectory_.calculateYaw();
        //纵向speed规划
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.process(&trajectory_);
    }

    if((stage_ == "Lap 2" || stage_ == "Lap 3") && is_adopting_global_method_)
    {
        generateLocalMapFromGlobalMap();
        //生成稠密的局部地图
        local_road_.left_boundry = lateral_planner_.process(perceived_road_.left_boundry);
        local_road_.right_boundry = lateral_planner_.process(perceived_road_.right_boundry);
        //生成道路中心线
        local_road_.generateCenterLine();
        trajectory_ = local_road_.center_line;
        //补充道路信息
        trajectory_.calculateS();
        trajectory_.calculateYaw();
        //纵向speed规划
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.process(&trajectory_);
    }
    
    if(stage_ == "Lap 4")
    {//deceleration
        local_road_.left_boundry = lateral_planner_.process(perceived_road_.left_boundry);
        local_road_.right_boundry = lateral_planner_.process(perceived_road_.right_boundry);
        local_road_.generateCenterLine();
        trajectory_ = local_road_.center_line;
        trajectory_.calculateS();
        trajectory_.calculateYaw();
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.emergencyStop(&trajectory_, 0.8 * 9.8);
    }
}

void FSARacetrackPlanning::generateLocalMapFromGlobalMap()
{
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