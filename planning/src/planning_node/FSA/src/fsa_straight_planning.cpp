#include "fsa_straight_planning.h"

bool FSAStraightPlanning::initalize()
{
    stage_ = "Accelerate";
    longitudinal_planner_.setMaxV(max_speed_);
    states_recording_.clear();
}

void FSAStraightPlanning::switchStage()
{
    for(auto cone : map_.yellow_cone)
    {
        if((pow((cone.position.x - state_.x), 2) + pow((cone.position.y - state_.y), 2)) < 5 * 5)
        {//如果黄色桩通出现在5m之内
            stage_ = "Decelerate";
            return;
        }
    }
}

void FSAStraightPlanning::runAlgorithm()
{
    switchStage();
    if(stage_ == "Accelerate")
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
        states_recording_.push_back(state_);
    }
    
    if(stage_ == "Decelerate")
    {
        //待完善
        if(!has_constructed_line_)
        {
            constructLine();
        }
        generateTrajectoryFromLine();
        longitudinal_planner_.setCurrentV(state_.vx);
        longitudinal_planner_.emergencyStop(&trajectory_, 0.8 * 9.8);
    }
}

void FSAStraightPlanning::constructLine()
{
    //input:
    //std::vector<planning::Vehicle> states_recording_ 
    
    //output:
    //double line_k_
    //double line_b_
    double sum_x = 0;
    double sum_y = 0;
    double sum_xy = 0;
    double sum_pow_x = 0;
    long long k ;
    line_k_ = 10.0;
    line_b_ = 10.0;
    int record_size = states_recording_.size();
    if (record_size <= 2){
        has_constructed_line_ = false;
        return;
    }

    for (auto state : states_recording_){
        sum_x += state.x;
        sum_y += state.y;
        sum_xy += state.x * state.y;
        sum_pow_x += state.x * state.x;
    }
    //防止溢出  x y单位为米  起始位置为零   
    //车辆从启动到终止  最多1km~2km   极限情况垂直y轴 2147483648 / 100000 
    //double you liuwei jingdu 
    //取k 最大为21474  y = kx +b 这样可以限制y在范围2147483648之内
    //行动范围有限  x y不太会超过double类型的限制  因此主要限制k不要溢出
    if ((sum_xy * record_size - sum_y * sum_x) / 100000 / (sum_pow_x * record_size - sum_x * sum_x) > 20000){
        line_k_ = 2000000000;
    }
    else {//y = kx +b ;  kx -y + b = 0 ; ax + by + c = 0; theta = arc(-b/a)
        line_k_ = (sum_xy * record_size - sum_y * sum_x) / (sum_pow_x * record_size - sum_x * sum_x);
    }
    line_b_ = (sum_pow_x * sum_y - sum_xy *sum_x) /(sum_pow_x * record_size - sum_x * sum_x);
    line_theta_ = std::atan (line_k_); 

    // line_theta_ = atan2(,);//弧度
    has_constructed_line_ = true;
}

void FSAStraightPlanning::generateTrajectoryFromLine()
{
    trajectory_.path.push_back(findNearestPointInLine());
    for(int i = 1; i < points_num_; i++)
    {   
        planning::PathPoint p;
        p.x = trajectory_.path[i - 1].x + unit_length_ * cos(line_theta_);
        p.y = trajectory_.path[i - 1].y + unit_length_ * sin(line_theta_);
        trajectory_.path.push_back(p);
    }
}

planning::PathPoint FSAStraightPlanning::findNearestPointInLine()
{
    planning::PathPoint result;
    int a, b, c;
    a = -line_k_;
    b = 1;
    c = -line_b_;
    //设直线方程为ax+by+c=0,点坐标为(m,n)
    result.x = (b * b * state_.x - a * b * state_.y - a * c) / (a * a + b * b);
    result.y = (a * a * state_.y - a * b * state_.x - b * c) / (a * a + b * b);
    return result;
}