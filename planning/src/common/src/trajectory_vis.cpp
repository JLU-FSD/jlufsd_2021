#include "trajectory_vis.h"


nav_msgs::Path planning::generatePathVisFromPath(const planning::Path & path)
{
    nav_msgs::Path path_vis;
    path_vis.poses.clear();
    for(int i = 0; i < path.path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path.path[i].x;
        pose.pose.position.y = path.path[i].y;
        pose.pose.position.z = 0;
        path_vis.poses.push_back(pose);
    }
    path_vis.header.frame_id = "map";
    return path_vis;
}
