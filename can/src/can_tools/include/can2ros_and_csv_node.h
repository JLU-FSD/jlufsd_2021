#pragma once
#include <yaml-cpp/yaml.h>
#include <socket_can_channel.hpp>
#ifdef ESD_DRIVERS
#include "esd_can_channel.hpp"
#endif
#include <memory>
#include "can_dbc.h"
#include "can_config_data_path.h"
#include <common_tool/csv_writer.h>
#include "auto_msgs/can_msg.h"
#include "ros/ros.h"
#include "can_pool.hpp"
#include <cmath>
#include <thread> // std::thread
#include <mutex>  // std::mutex
#include "auto_ros_base_path.h"

std::string dec2hex_str(int i, int width)
{
	std::stringstream ioss; //定义字符串流
	std::string s_temp;		//存放转化后字符
	ioss << std::hex << i;	//以十六制形式输出
	ioss >> s_temp;

	if (width > s_temp.size())
	{
		std::string s_0(width - s_temp.size(), '0'); //位数不够则补0
		s_temp = s_0 + s_temp;						 //合并
	}

	std::string s = s_temp.substr(s_temp.length() - width, s_temp.length()); //取右width位
	return s;
}
class Can2RosAndCsvNode
{
public:
	Can2RosAndCsvNode() = delete;
	Can2RosAndCsvNode(std::string can_node_yaml_file_name);
	void step();
	ros::NodeHandle nh;
	std::shared_ptr<ros::Rate> loop_rate_ptr_;
	double rate_;
	std::shared_ptr<io::CSVWriter> log_writer_ptr_;
	bool read_can_pool_sig(const unsigned int can_id, const std::string sig_name, double &sig_value);
	void can_cb(can_frame frame);

private:
	std::vector<std::shared_ptr<can::BaseCanChannel>> can_channel_vec_;
	std::map<std::string, auto_msgs::can_msg> can_msgs_ros_map_;
	std::map<std::string, ros::Publisher> pub_map_;
	std::map<std::string, std::shared_ptr<io::CSVWriter>> csv_map_;
	CanMsgsPool can_msgs_pool_;
	void init_map();
	std::mutex mtx_;
};