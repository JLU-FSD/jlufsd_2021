#include "smoother_common.h"
Smoother::Smoother() {}

void Smoother::input(const planning::Path &_original_traj)
{
	original_traj_ = _original_traj;
}

planning::Path Smoother::output()
{
	return smoothed_traj_;
}

planning::Path Smoother::process(planning::Path &_input_path)
{
	input(_input_path);
	if(original_traj_.path.size() <= 2)
	{
        return original_traj_;
	}
    smooth();
	return output();
}