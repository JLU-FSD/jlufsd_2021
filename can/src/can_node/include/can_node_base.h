#pragma once
#include <yaml-cpp/yaml.h>
#include <socket_can_channel.hpp>
#ifdef ESD_DRIVERS
#include "esd_can_channel.hpp"
#endif
#include <memory>
#include "can_dbc.h"
#include "can_config_data_path.h"
#include "ros/ros.h"
#include "can_pool.hpp"
#include <cmath>
#include <thread> // std::thread
#include <mutex>  // std::mutex
#include "can/test_uiConfig.h"
#include <dynamic_reconfigure/server.h>

class CanNodeBase
{
public:
	CanNodeBase() = delete;
	CanNodeBase(std::string can_node_yaml_file_name);
	virtual void step() = 0;
	ros::NodeHandle nh_;
	std::shared_ptr<ros::Rate> loop_rate_ptr_;
	double rate_;
	bool read_can_pool_sig(const unsigned int can_id, const std::string sig_name, double &sig_value);
	void can_cb(can_frame frame);
	virtual void register_custom_can_cb(std::function<void(can_frame)> can_cb_fun);

protected:
	std::vector<std::shared_ptr<can::BaseCanChannel>> can_ptr_vec_;
	CanMsgsPool can_msgs_pool_;
	std::mutex can_cb_mtx_;
	YAML::Node can_yaml_;

	dynamic_reconfigure::Server<can::test_uiConfig> server_;
	dynamic_reconfigure::Server<can::test_uiConfig>::CallbackType f_;
	void test_ui_callback(can::test_uiConfig &config, uint32_t level);
	can::test_uiConfig test_ui_par_;
};