#include <ros/ros.h>
#include "planning_handle.hpp"


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "planning_fsa");
    ros::NodeHandle nodeHandle("~");
    PlanningHandle planningHandle(nodeHandle);
    ros::Rate loop_rate(planningHandle.getNodeRate());

    while (ros::ok()) {

        planningHandle.run();
        ros::spinOnce();                // Keeps node alive basically
        loop_rate.sleep();              // Sleep for loop_rate
    }
    return 0;
}