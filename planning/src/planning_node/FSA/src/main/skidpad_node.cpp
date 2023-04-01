#include "fsa_skidpad_planning.h"
#include <ros/ros.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "planning");
    FSASkidpadPlanning core;
    core.initalize();
    ros::Rate loop_rate(core.getLoopRate());
    while (ros::ok()) 
    {
        core.loopProc();
        ros::spinOnce();                
        loop_rate.sleep();              
    }
    return 0;
}