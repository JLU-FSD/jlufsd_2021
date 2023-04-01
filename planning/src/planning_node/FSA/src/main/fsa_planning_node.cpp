#include "fsa_planning_node.h"
#include "fsa_planning.h"
#include <ros/ros.h>

FSAPlanningNode::FSAPlanningNode()
{
    temp_nh_ptr_ = std::make_shared<ros::NodeHandle>();
    mission_subscriber_ = temp_nh_ptr_->subscribe("mission_topic", 1, &FSAPlanningNode::missionCallback, this);
}

void FSAPlanningNode::missionCallback(const auto_msgs::Mission &msg)
{
    if(msg.mission == "Straight" || msg.mission == "Skidpad" || msg.mission == "Racetrack" || msg.mission == "Other")
    {
        has_recieved_mission_ = true;
        if(mission_ != msg.mission)
        {
            mission_ = msg.mission;
            mission_changed_ = true;
            std::cout<< "Plannning receive new mission!" <<std::endl;
        }
    }
}

void FSAPlanningNode::waiteForMisson()
{
    while(!has_recieved_mission_)
    {
        ros::spinOnce();
        if(has_recieved_mission_)
        {
            break;
        }
        std::cout<< " Planning is waiting for Mission!" <<std::endl;
    }
}

std::shared_ptr<FSAPlanning> FSAPlanningNode::planningFactory()
{
    if(mission_ == "Straight")
    {
        return std::make_shared<FSAStraightPlanning>();
    }
    if(mission_ == "Skidpad")
    {
        return std::make_shared<FSASkidpadPlanning>();
    }
    if(mission_ == "Racetrack")
    {
        return std::make_shared<FSARacetrackPlanning>();
    }
    if(mission_ == "Other")
    {
        return nullptr;
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "planning");
    FSAPlanningNode node;
    node.waiteForMisson();
    node.core_ = node.planningFactory();
    if(node.core_)node.core_->initalize();

    ros::Rate loop_rate(node.core_->getLoopRate());
    while (ros::ok()) 
    {
        if(node.mission_changed_)
        {
            std::shared_ptr<FSAPlanning> core_ = node.planningFactory();
            if(node.core_)node.core_->initalize();
            node.mission_changed_ = false;
        }
        if(node.core_)node.core_->loopProc();
        ros::spinOnce();                
        loop_rate.sleep();              
    }
    return 0;
}