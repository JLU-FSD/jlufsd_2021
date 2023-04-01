#include "can_node_base.h"
#include "geometry_msgs/Twist.h"
namespace auto_ros
{
namespace can
{
class CanTesterNode : public CanNodeBase
{
public:
	CanTesterNode() = delete;
	CanTesterNode(std::string can_node_yaml_file_name);
	virtual void step();
	void cb_keyboard(const geometry_msgs::Twist &msg);
	void steering_to_can(double tire_angle);

private:
	geometry_msgs::Twist twist_;

	ros::Subscriber twist_sub_ =
		nh_.subscribe("turtle1/cmd_vel", 3, &CanTesterNode::cb_keyboard, this);
	double steering_angle_ = 0;
	unsigned char steer_live_counter_ = 0;
	unsigned char live_counter_timer_ = 0;
	unsigned char steer_live_counter_max_ = 15;
	void my_can_callback(can_frame frame);
};

} // namespace can
} // namespace auto_ros