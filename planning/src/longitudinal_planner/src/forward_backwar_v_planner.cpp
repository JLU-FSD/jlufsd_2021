#include "forward_backwar_v_planner.h"

ForwardBackwardVPlanner::ForwardBackwardVPlanner(){}

//计算函数
void ForwardBackwardVPlanner::calculateVConstrainedByCur()
{//计算由曲率决定的最大速度
    for(int i = 0; i < traj_ptr_->path.size(); i++)
    {
        traj_ptr_->path[i].vx = planning::box(sqrt( f_ * g_ / abs(traj_ptr_->path[i].cur) ),
                                            0, max_v_);
    }
}

void ForwardBackwardVPlanner::calculateVConstrainedByAcceleration()
{//前向计算由驱动力限制决定的最大速度
    traj_ptr_->path[0].vx = current_v_;
    for (int i = 0; i < traj_ptr_->path.size() - 1; i++)
    {
        if (traj_ptr_->path[i + 1].vx > traj_ptr_->path[i].vx)
        {
			traj_ptr_->path[i].ax = sqrt(pow(f_ * g_, 2) - pow(traj_ptr_->path[i].vx, 4) * pow(traj_ptr_->path[i + 1].cur, 2));
            traj_ptr_->path[i + 1].vx = sqrt(pow(traj_ptr_->path[i].vx, 2) + 2 * traj_ptr_->path[i].ax * (traj_ptr_->path[i + 1].s - traj_ptr_->path[i].s));
        }
    }
}

void ForwardBackwardVPlanner::calculateVConstrainedByDeceleration()
{//后向计算由制动力限制决定的最大速度
    for (int i = traj_ptr_->path.size() - 1; i > 0; i--)
    {
        if (traj_ptr_->path[i -1 ].vx > traj_ptr_->path[i].vx)
        {
            traj_ptr_->path[i].ax = -sqrt(pow(f_* g_, 2) - pow(traj_ptr_->path[i].vx, 4) * pow(traj_ptr_->path[i -1].cur, 2));
            traj_ptr_->path[i - 1].vx = sqrt(pow(traj_ptr_->path[i].vx, 2) - 2 * traj_ptr_->path[i].ax * (traj_ptr_->path[i].s - traj_ptr_->path[i - 1].s));
        }
    }
}

//过程控制函数
bool ForwardBackwardVPlanner::initalize()
{
    if(!traj_ptr_->has_s)
    {
        traj_ptr_->calculateS();
        return true;
    }
    return true;
}
void ForwardBackwardVPlanner::plan()
{
    calculateVConstrainedByCur();
    if(consider_acceleration_constrain_)
    {
        calculateVConstrainedByAcceleration();
        calculateVConstrainedByDeceleration();
    }
}