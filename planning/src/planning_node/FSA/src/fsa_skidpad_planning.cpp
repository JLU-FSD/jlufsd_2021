#include "fsa_skidpad_planning.h"

bool FSASkidpadPlanning::initalize()
{
    stage_ = "Start";
    longitudinal_planner_.setMaxV(max_speed_);
    is_near_high_yellow_cone_ = false;
    has_recieved_yellow_cones_ = false;
}

void FSASkidpadPlanning::switchStage()
{
    stage_need_to_be_updated_ = stageNeedToBeUpdated();
    if(stage_ == "Start")
    {
        if(has_recieved_yellow_cones_)
        {
            stage_ = "Entrance";
        }
    }
    if(stage_ == "Entrance")
    {
        if( stage_need_to_be_updated_)
        {
            stage_ = "In 8 Shape";
            lap_num_ = 1;
        }
    }
    if(stage_ == "In 8 Shape")
    {
        if(lap_num_ < 4 && stage_need_to_be_updated_)
        {
            lap_num_++;
        }
        if(lap_num_ == 4)
        {
            if( stage_need_to_be_updated_)
            {
                has_recieved_yellow_cones_ = false;
                stage_ == "Export";
            }
        }
    }
}

bool FSASkidpadPlanning::stageNeedToBeUpdated()
{//刚刚接近黄色高桩通时更新状态
    if(is_near_high_yellow_cone_ == true)
    {
        if(map_.yellow_high_cone.size() == 0)
        {
            if(++no_yellow_high_cone_count_ > 3 * getLoopRate())
            {//3s
                is_near_high_yellow_cone_ = false;
            }
        }
    }
    else
    {
        no_yellow_high_cone_count_ = 0;
        int near_yellow_high_cone_num = 0;
        for(auto cone : map_.yellow_high_cone)
        {
            if((pow((cone.position.x - state_.x), 2) + pow((cone.position.y - state_.y), 2)) < 3 * 3)
            {//如果黄色高桩通出现在3m之内
                near_yellow_high_cone_num++;
            }
        }
        if(near_yellow_high_cone_num == 2)
        {
            is_near_high_yellow_cone_ = true;
            return true;
        }
    }
    return false;
}

void FSASkidpadPlanning::runAlgorithm()
{
    longitudinal_planner_.setCurrentV(state_.vx);
    switchStage();

    if(stage_ == "Start")
    {
        std::cout<< "Has not recieved yellow cones nearby the entrance!"<<std::endl;
        has_recieved_yellow_cones_ = getYellowCones();
    }

    if(stage_ == "Entrance")
    {
        enter8Shape();
    }

    if(stage_ == "In 8 Shape")
    {
        run8Shape();
    }

    if(stage_ == "Export")
    {
        leave8Shape();
    }
}

void FSASkidpadPlanning::enter8Shape()
{
    generateTrajectoryFrom(entrance_path_);
    longitudinal_planner_.setCurrentV(state_.vx);
    longitudinal_planner_.process(&trajectory_);
}


void FSASkidpadPlanning::run8Shape()
{
    if(!getReferenceCones())
    {
        std::cout<< "Can not get the reference cones!"<<std::endl;
        return;
    }
    if(!getCircleCenter())
    {
        std::cout<< "Can not get the circle center!"<<std::endl;
        return;
    }
    generateCircleTrajectory();
    generateTrajectoryFrom(circle_trajectory_);
    longitudinal_planner_.setCurrentV(state_.vx);
    longitudinal_planner_.process(&trajectory_);
}

void FSASkidpadPlanning::leave8Shape()
{
    if(!has_recieved_yellow_cones_)
    {
        generateTrajectoryFrom(entrance_path_);
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.process(&trajectory_);
        has_recieved_yellow_cones_ = getYellowCones();
    }
    else
    {
        generateTrajectoryFrom(export_path_);
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.process(&trajectory_);
    }
}

void FSASkidpadPlanning::generateTrajectoryFrom(planning::Path path)
{
    const auto it_nearest = std::min_element(path.path.begin(), path.path.end(),
                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.x, state_.y - a.y);
                                        const double db = std::hypot(state_.x - b.x, state_.y - b.y);
                                        return da < db;
                                    });
    trajectory_.path.clear();
    for(auto it = it_nearest; it != path.path.end(); it++)
    {
        trajectory_.path.push_back(*it);
    }
}

void FSASkidpadPlanning::generateTrajectoryFrom(planning::Path current_path, planning::Path next_path)
{
    const auto it_nearest = std::min_element(current_path.path.begin(), current_path.path.end(),
                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.x, state_.y - a.y);
                                        const double db = std::hypot(state_.x - b.x, state_.y - b.y);
                                        return da < db;
                                    });
    trajectory_.path.clear();
    for(auto it = it_nearest; it != current_path.path.end(); it++)
    {
        trajectory_.path.push_back(*it);
    }
    for(auto it = next_path.path.end(); it != next_path.path.end(); it++)
    {
        trajectory_.path.push_back(*it);
    }
}

void FSASkidpadPlanning::getEntranceLine()
{
    entrance_path_.path.clear();
    double x0; //入口起始位置
    double y0; //入口起始位置
    double th;
    double th1;
    double th2;
    double x;
    double y;
    int n;
    n = entrance_path_length_/entrance_path_precision_;
    x0 = (yellow_cones_[0].position.x + yellow_cones_[1].position.x)/2;
    y0 = (yellow_cones_[0].position.y + yellow_cones_[1].position.y)/2;

    x = yellow_cones_[0].position.x - yellow_cones_[1].position.x;
    y = yellow_cones_[0].position.y - yellow_cones_[1].position.y;
    th = atan2(y,x);
    th = th + M_PI/2;
    th1 = th -  M_PI/2;
    x = x0 - state_.x;
    y = y0 - state_.y;
    th2 = atan2(y,x);
    th = fabs(th - th2);
    th1 = fabs(th1 - th2);
    if(th < th1)
    {
        th = th;
    }
    else
    {
        th = th1;
    }
    for(int i=0; i<n; i++)
    {
        planning::PathPoint p;
        p.x = x0 + i*cos(th);
        p.y = y0 + i*sin(th);
        entrance_path_.path.push_back(p);
    }
}

void FSASkidpadPlanning::getExportLine()
{
    export_path_.path.clear();
    double x0;
    double y0;
    double th;
    double x;
    double y;
    double len;
    int n;
    x0 = (yellow_cones_[0].position.x + yellow_cones_[1].position.x)/2;
    y0 = (yellow_cones_[0].position.y + yellow_cones_[1].position.y)/2;
    len = sqrt((x0 - state_.x)*(x0 - state_.x) + (y0 - state_.y)*(y0 - state_.y));
    n = len/export_path_precision_;
    x = x0 - state_.x;
    y = y0 - state_.y;
    th = atan2(y,x);
    for(int i=0; i<n; i++)
    {
        planning::PathPoint p;
        p.x = state_.x + i*cos(th);
        p.y = state_.y + i*sin(th);
        export_path_.path.push_back(p);
    }
}

bool FSASkidpadPlanning::getYellowCones()
{
    if(map_.yellow_cone.size() <= 2)
    {
        return false;
    }
    double delta_x, delta_y, distance;
    for(int i = 0; i < map_.yellow_cone.size(); i++)
    {
        for(int j = 0; j < map_.yellow_cone.size(); j++)
        {
            if(i == j)
            {
                continue;
            }
            delta_x = map_.yellow_cone[i].position.x - map_.yellow_cone[j].position.x;
            delta_y = map_.yellow_cone[i].position.y - map_.yellow_cone[j].position.y;
            distance = pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);
            if(distance >= yellow_cones_distance_ - yellow_cones_distance_error_ && distance <= yellow_cones_distance_ + yellow_cones_distance_error_)
            {
                yellow_cones_.clear();
                yellow_cones_.push_back(map_.yellow_cone[i]);
                yellow_cones_.push_back(map_.yellow_cone[j]);
                getEntranceLine();
                return true;
            }
        }
    }
    return false;
}
bool FSASkidpadPlanning::getReferenceCones()
{
    reference_cones_.clear();
    if(lap_num_ == 1 || lap_num_ == 2)
    {
        if(map_.blue_cone.size() < 2)
        {
            std::cout<<"Don't have enough blue cone!"<<std::endl;
            return false;
        }
        const auto it_nearest_cone = std::min_element(map_.blue_cone.begin(), map_.blue_cone.end(),
                                    [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.position.x, state_.y - a.position.y);
                                        const double db = std::hypot(state_.x - b.position.x, state_.y - b.position.y);
                                        return da < db;
                                    });
        if(sqrt(pow(it_nearest_cone->position.x - state_.x, 2) + pow(it_nearest_cone->position.y - state_.y, 2)) > 5)
        {
            std::cout<<"The nearest blue cone is too far from ego vehicle!"<<std::endl;
            return false;
        }
        const auto it_next_cone = std::min_element(map_.blue_cone.begin(), map_.blue_cone.end(),
                                    [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                    {
                                        const double da = std::hypot(it_nearest_cone->position.x - a.position.x, it_nearest_cone->position.y - a.position.y);
                                        const double db = std::hypot(it_nearest_cone->position.x - b.position.x, it_nearest_cone->position.y - b.position.y);
                                        return da < db;
                                    });
        if(sqrt(pow(it_nearest_cone->position.x - it_next_cone->position.x, 2) + pow(it_nearest_cone->position.y - it_next_cone->position.y, 2)) > 5)
        {
            std::cout<<"The next cone of the nearest blue cone is too far from ego vehicle!"<<std::endl;
            return false;
        }
        reference_cones_.push_back(*it_nearest_cone);
        reference_cones_.push_back(*it_next_cone);
    }
    if(lap_num_ == 3 || lap_num_ == 4)
    {
        if(map_.red_cone.size() < 2)
        {
            std::cout<<"Don't have enough red cone!"<<std::endl;
            return false;
        }
        const auto it_nearest_cone = std::min_element(map_.red_cone.begin(), map_.red_cone.end(),
                                    [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                    {
                                        const double da = std::hypot(state_.x - a.position.x, state_.y - a.position.y);
                                        const double db = std::hypot(state_.x - b.position.x, state_.y - b.position.y);
                                        return da < db;
                                    });
        if(sqrt(pow(it_nearest_cone->position.x - state_.x, 2) + pow(it_nearest_cone->position.y - state_.y, 2)) > 5)
        {
            std::cout<<"The nearest red cone is too far from ego vehicle!"<<std::endl;
            return false;
        }
        const auto it_next_cone = std::min_element(map_.red_cone.begin(), map_.red_cone.end(),
                                    [&](const auto_msgs::Cone &a, const auto_msgs::Cone &b) 
                                    {
                                        const double da = std::hypot(it_nearest_cone->position.x - a.position.x, it_nearest_cone->position.y - a.position.y);
                                        const double db = std::hypot(it_nearest_cone->position.x - b.position.x, it_nearest_cone->position.y - b.position.y);
                                        return da < db;
                                    });
        if(sqrt(pow(it_nearest_cone->position.x - it_next_cone->position.x, 2) + pow(it_nearest_cone->position.y - it_next_cone->position.y, 2)) > 5)
        {
            std::cout<<"The next cone of the nearest red cone is too far from ego vehicle!"<<std::endl;
            return false;
        }
        reference_cones_.push_back(*it_nearest_cone);
        reference_cones_.push_back(*it_next_cone);
    }
}   

bool FSASkidpadPlanning::getCircleCenter()
{
    double x0;
    double y0;
    double th;
    double x;
    double y;
    double x1;
    double x2;
    double y1;
    double y2;
    double dis1;
    double dis2;
    int n;
    x0 = (reference_cones_[0].position.x + reference_cones_[1].position.x)/2;
    y0 = (reference_cones_[0].position.y + reference_cones_[1].position.y)/2;
    x = reference_cones_[0].position.x - reference_cones_[1].position.x;
    y = reference_cones_[0].position.y - reference_cones_[1].position.y;
    th = atan2(y,x);
    x1 = x0 + cone_radius_*cos(th);
    y1 = y0 + cone_radius_*sin(th);
    x2 = x0 - cone_radius_*cos(th);
    y2 = y0 - cone_radius_*sin(th);
    dis1 = sqrt((x1 - state_.x)*(x1 - state_.x)+(y1 - state_.y)*(y1 - state_.y));
    dis2 = sqrt((x2 - state_.x)*(x2 - state_.x)+(y2 - state_.y)*(y2 - state_.y));
    if(dis1 > dis2)
    {
        circle_center_.x = x1;
        circle_center_.y = y1;
    }
    else
    {
        circle_center_.x = x2;
        circle_center_.y = y2;
    }
}

bool FSASkidpadPlanning::generateCircleTrajectory()
{
    circle_trajectory_.path.clear();
    double current_angle = std::atan2(state_.y - circle_center_.y, state_.x - circle_center_.x);
    if(lap_num_ == 1 || lap_num_ == 2)
    {//顺时针
        for(double angle = current_angle; angle <= current_angle - M_PI; angle -= skidpad_angle_precision_)
        {
            planning::PathPoint p;
            p.x = circle_center_.x + std::cos(angle);
            p.y = circle_center_.y + std::sin(angle);
            circle_trajectory_.path.push_back(p);
        }
    }
    if(lap_num_ == 3 || lap_num_ == 4)
    {//逆时针
        for(double angle = current_angle; angle <= current_angle + M_PI; angle += skidpad_angle_precision_)
        {
            planning::PathPoint p;
            p.x = circle_center_.x + std::cos(angle);
            p.y = circle_center_.y + std::sin(angle);
            circle_trajectory_.path.push_back(p);
        }
    }
}