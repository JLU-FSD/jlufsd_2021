#pragma once
#include "planning_common.h"
class VelocityPlanner
{
public:
    VelocityPlanner();
    planning::PathPtr           traj_ptr_;
    void                        process(planning::PathPtr _traj_ptr);
    void                        emergencyStop(planning::PathPtr _traj_ptr, double deceleration);

    virtual bool                initalize() = 0;//虚方法=0表示子类必须实现
    virtual void                plan() = 0;//虚方法=0表示子类必须实现

    void                        setCurrentV(const double & _v){current_v_ = _v;/* m/s */};
    void                        setMaxV(const double & _v){max_v_ = _v;/* m/s */};

    double                      current_v_ = 0;
    double                      max_v_ = 120/3.6;
};