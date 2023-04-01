#pragma once
#include "longitudinal_common.h"

class ForwardBackwardVPlanner : public VelocityPlanner
{
public:
    ForwardBackwardVPlanner();
	
    planning::VehicleParam      vp_;
    bool                        consider_acceleration_constrain_ = true;
    double                      f_ = 0.8;//摩擦系数
    double                      g_ = 9.8;//重力加速度
    double                      v_current_ = 36/3.6;//当前车速m/s
    double                      v_limit_ = 120/3.6;//最高限速m/s
    // double                      max_tire_force_ = f_ * g_ ;//最大总的轮胎力

    
    //检查函数
    bool                    checkS();

    //计算函数
    void                    calculateVConstrainedByCur();//计算由曲率决定的最大速度
    void                    calculateVConstrainedByAcceleration();//前向计算由制动力限制决定的最大速度
    void                    calculateVConstrainedByDeceleration();//后向计算由驱动力限制决定的最大速度

    //过程控制函数
    bool                    initalize();
    void                    plan();
    
private: 

};