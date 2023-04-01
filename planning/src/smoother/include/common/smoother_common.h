#pragma once
#include "trajectory.h"

class Smoother
{
public:
    Smoother();
    void input(const planning::Path &_original_traj);
    planning::Path output();
    planning::Path process(planning::Path &_original_traj);
    virtual void smooth() = 0;//虚方法=0表示子类必须实现
	
    planning::Path original_traj_;
    planning::Path smoothed_traj_;
};