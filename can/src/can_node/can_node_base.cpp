#include "can_node_base.h"
#include "common_tool/angle_helper.h"

CanNodeBase::CanNodeBase(std::string can_node_yaml_file_name)
{
	f_ = boost::bind(&CanNodeBase::test_ui_callback, this, _1, _2);
	server_.setCallback(f_);
	can_yaml_ = YAML::LoadFile(can_node_yaml_file_name);
	rate_ = can_yaml_["rate"].as<double>();
	loop_rate_ptr_ = std::make_shared<ros::Rate>(rate_);
	//初始化can总线
	std::string folderpath = can_dbcs_path;
	std::vector<std::string> files;
	getAllFilesInFolder(folderpath, &files);
	for (std::string &file : files)
	{
		//std::cout << file << std::endl;
		dbc_analysis::DbcAnalysis::getInstance()->addOneDbcFile(file);
	}
	dbc_analysis::DbcAnalysis::getInstance()->analysisFiles();
	dbc_analysis::DbcAnalysis::getInstance()->printMessages();
	can_msgs_pool_.init(dbc_analysis::DbcAnalysis::getInstance()->getMessages());
	can_msgs_pool_.set_all_dog(100, 1, 0, 17, 30);
	std::function<void(can_frame)> can_cb_fun = std::bind(&CanNodeBase::can_cb, this, std::placeholders::_1);
	std::vector<std::string> can_yaml_vec = can_yaml_["can_config_files"].as<std::vector<std::string>>();

	for (int i = 0; i < can_yaml_vec.size(); i++)
	{

		if (can_yaml_["can_type"].as<std::string>() == "SocketCAN")
		{
			can_ptr_vec_.push_back(std::make_shared<can::SocketCANChannel>(can_config_data_path + "socket_can_config/" + can_yaml_vec[i]));
		}
		else
		{
#ifdef ESD_DRIVERS
			can_ptr_vec_.push_back(std::make_shared<can::EsdCanChannel>(can_config_data_path + "esd_can_config/" + can_yaml_vec[i]));
			std::shared_ptr<can::BaseCanChannel> temp_ptr = can_ptr_vec_[i];
			std::vector<unsigned int> ids_vec = dbc_analysis::DbcAnalysis::getInstance()->ids();
			dynamic_cast<can::EsdCanChannel *>(can_ptr_vec_[i].get())->add_ids(ids_vec); //legal
#else
			std::cout << "\033[31m Only SocketCAN is supported. \033[0m" << std::endl;
			throw 1;
#endif
		}
		can_ptr_vec_[i]->register_asyn_read_cb(can_cb_fun);
		can_ptr_vec_[i]->start_asyn_read();
	}
}
//position heading and velocity
void CanNodeBase::can_cb(can_frame frame)
{
	can_cb_mtx_.lock();
	can_msgs_pool_.write_pool(frame);
	can_cb_mtx_.unlock();
}
void CanNodeBase::register_custom_can_cb(std::function<void(can_frame)> can_cb_fun)
{
	for (int index = 0; index < can_ptr_vec_.size(); index++)
	{
		can_ptr_vec_[index]->register_asyn_read_cb(can_cb_fun);
	}
	return;
}
bool CanNodeBase::read_can_pool_sig(const unsigned int can_id, const std::string sig_name, double &sig_value)
{
	std::map<std::string, double> can_sigs;
	if (can_msgs_pool_.read_frame_sigs(can_id, can_sigs, false) == false)
	{
		std::cout << "\033[31m CanNode::read_can_pool_sig failed, \033[0m" << std::endl;
		return false;
	}
	sig_value = can_sigs[sig_name];
	return true;
}

void CanNodeBase::test_ui_callback(can::test_uiConfig &config, uint32_t level)
{
	test_ui_par_ = config;
	ROS_INFO("test_ui Request: %s %f %f",
			 config.isAuto ? "True" : "False", config.man_tire_angle, config.man_acc);
}