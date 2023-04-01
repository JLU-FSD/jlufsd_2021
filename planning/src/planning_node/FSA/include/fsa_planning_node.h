#include <string>
#include "fsa_skidpad_planning.h"
#include "fsa_straight_planning.h"
#include "fsa_racetrack_planning.h"
#include "auto_msgs/Mission.h"

class FSAPlanningNode
{
public:
    FSAPlanningNode();
    bool                                mission_changed_ = false;
    std::shared_ptr<FSAPlanning>        core_;

    void                                waiteForMisson();
    std::shared_ptr<FSAPlanning>        planningFactory();
    
private:
    std::shared_ptr<ros::NodeHandle>    temp_nh_ptr_;
    bool                                has_recieved_mission_ = false;
    ros::Subscriber                     mission_subscriber_;
    std::string                         mission_;

    void                                missionCallback(const auto_msgs::Mission &msg);
};

