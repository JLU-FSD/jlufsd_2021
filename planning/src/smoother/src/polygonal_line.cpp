#include "polygonal_line.h"

void PolygonalLine::smooth()
{
    original_traj_.calculateS();
    original_traj_.calculateYaw();
    pieces_num_ = original_traj_.path.size() - 1;
    smoothed_traj_.path.clear();
    for(int i = 0; i < pieces_num_; i++)
    {
        double distance = original_traj_.path[i + 1].s - original_traj_.path[i].s;
        for(double s = 0; s < distance; s += precision_)
        {
            planning::PathPoint p;
            p.x = original_traj_.path[i].x + s * cos(original_traj_.path[i].yaw);
            p.y = original_traj_.path[i].y + s * sin(original_traj_.path[i].yaw);
            smoothed_traj_.path.push_back(p);
        }
    }
    smoothed_traj_.path.push_back(original_traj_.path.back());
}