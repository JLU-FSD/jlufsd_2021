#include "longitudinal_common.h"

VelocityPlanner::VelocityPlanner() {}

void VelocityPlanner::process(planning::PathPtr _traj_ptr)
{
	traj_ptr_ = _traj_ptr;
	if(_traj_ptr->path.size() <= 2)
	{
        return;
	}
	if(!initalize())
	{
        return;
	}
    plan();
	traj_ptr_->has_v = true;
    traj_ptr_->has_a = true;
}

void VelocityPlanner::emergencyStop(planning::PathPtr _traj_ptr, double deceleration)
{
	traj_ptr_ = _traj_ptr;
	if(!traj_ptr_->has_s)
	{
		traj_ptr_->calculateS();
	}
	traj_ptr_->path[0].ax = - deceleration;
	traj_ptr_->path[0].vx = current_v_;
	for(int i = 1; i < traj_ptr_->path.size() + 1; i++)
	{ 
		traj_ptr_->path[i].ax = - deceleration;
		traj_ptr_->path[i].vx = pow(pow( traj_ptr_->path[i-1].vx, 2) + 2 * traj_ptr_->path[i-1].ax * (traj_ptr_->path[i].s - traj_ptr_->path[i-1].s), 0.5);
	}
	traj_ptr_->has_v = true;
    traj_ptr_->has_a = true;
}