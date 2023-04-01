#include "can_tester_node.h"
namespace auto_ros
{
namespace can
{

CanTesterNode::CanTesterNode(std::string can_node_yaml_file_name) : CanNodeBase(can_node_yaml_file_name) //继承can_node_base
{
	std::function<void(can_frame)> can_cb_fun = std::bind(&CanTesterNode::my_can_callback, this, std::placeholders::_1);
	register_custom_can_cb(can_cb_fun);
}

void CanTesterNode::step() //实现can_node_base的纯虚方法
{

	ros::spinOnce();
	steering_to_can(steering_angle_); //自定义功能，里面含有标准can消息自动打包并发送的例子
	//自动解析和读取数据
	std::map<std::string, double> pose_can_sigs;
	//bool read_frame_sigs(unsigned int can_id, std::map<std::string, double> &can_sigs, bool hunger = false)
	if (can_msgs_pool_.read_frame_sigs(0x706, pose_can_sigs, false) == false)
	{
		std::cout << "\033[31m pose feedback failed\033[0m" << std::endl;
	}
	else
	{
		std::cout << pose_can_sigs["Yaw"] << std::endl;
	}
	//
	loop_rate_ptr_->sleep();
}
void CanTesterNode::steering_to_can(double tire_angle) //实现了标准can消息的打包与发送
{
	tire_angle += 0.0;
	if (steer_live_counter_ >= steer_live_counter_max_)
	{
		live_counter_timer_ = 0;
	}
	steer_live_counter_ = live_counter_timer_ / 1;
	live_counter_timer_++;
	std::vector<double> steering_cmd(10, 0);
	steering_cmd[0] = 0;					 //check_sum
	steering_cmd[1] = std::fabs(tire_angle); //steering angle
	steering_cmd[2] = 1;					 //angel sign
	steering_cmd[3] = 10.0;					 //steer deg/s
	steering_cmd[4] = 1;					 //steer sign
	steering_cmd[5] = steer_live_counter_;	 //live_counter 0-15 ?
	steering_cmd[6] = 0;					 //control mode 0:angle ctr, 1:speed ctr , 2: double ctr
	steering_cmd[7] = 1;					 //enable ?
	steering_cmd[8] = 0;					 //reset ?
	steering_cmd[9] = 0;					 //carnum ?
	can_frame frame_0x060 = {0};

	//can消息自动打包
	//int packCanmsg(const Message &m, const std::vector<double> &signals_value, can_frame &msg);
	can_util::packCanmsg(dbc_analysis::DbcAnalysis::getInstance()->getMessages()[0x060], steering_cmd, frame_0x060);
	//can消息发送，如果是发送裸消息也可直接发送而不经过上面步骤的自动打包

	can_ptr_vec_[can_yaml_["can_send_channel"].as<int>()]->write_frame(frame_0x060); //指定发送的通道和数据，这里其实就是发送的裸数据
}
void CanTesterNode::cb_keyboard(const geometry_msgs::Twist &msg)
{
	twist_ = msg;
	steering_angle_ += twist_.angular.z;
}
void CanTesterNode::my_can_callback(can_frame frame) //对于非标准can消息，可以使用如下方式接受裸消息，然后手动解析
{
	if (frame.can_id = 0x101)
	{
		if (frame.data[0] = 1)
		{
			//后面七个字节是左边电机的
		}
		for (int index = 0; index < 8; index++)
		{
			std::cout << "data[" << index << "] is:" << frame.data[index] << std::endl;
		}
	}
}

} // namespace can
} // namespace auto_ros