#include "ros/ros.h"
#include <auto_msgs/chassis.h>
 
int main(int argc, char **argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle n;
    //topic的名称为chatter,1000为缓冲区，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。 
    ros::Publisher chatter = n.advertise<auto_msgs::chassis>("chassis",1000);
    ros::Rate loop_rate(10); //发布频率为10FPS
 
    while(n.ok())
    {
        auto_msgs::chassis chassis_topic;

        chassis_topic.gear_location=2;
 
        //发布消息
        chatter.publish(chassis_topic);
        ros::spinOnce(); //可用回调函数
        loop_rate.sleep();//休眠一段时间以使得发布频率为 10Hz。
  
    }
    return 0;
}
