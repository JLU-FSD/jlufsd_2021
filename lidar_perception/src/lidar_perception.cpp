#include "pcl_lidar_perception.h"

int 
main(int argc,char **argv)
{
    ros::init(argc,argv,"perception");
    ros::NodeHandle nodehandle;
    // ros::Rate r(10);
    LidarPerception core(nodehandle);
    return 0;
}