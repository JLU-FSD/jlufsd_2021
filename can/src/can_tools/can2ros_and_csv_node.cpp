#include "can2ros_and_csv_node.h"
#include "common_tool/angle_helper.h"
#include "common_tool/path.h"
#include "auto_ros_time.h"
using namespace auto_ros::common_tool;

Can2RosAndCsvNode::Can2RosAndCsvNode(std::string can_node_yaml_file_name)
{
	YAML::Node can_yaml = YAML::LoadFile(can_node_yaml_file_name);
	rate_ = can_yaml["rate"].as<double>();
	loop_rate_ptr_ = std::make_shared<ros::Rate>(rate_);
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
	can_msgs_pool_.set_all_dog(10, 1, 0, 17, 30);
	can_msgs_pool_.set_one_dog(0x710, 100 * rate_, 1, 0, 17, 30);
	std::function<void(can_frame)> can_cb_fun = std::bind(&Can2RosAndCsvNode::can_cb, this,
														  std::placeholders::_1);
	std::vector<std::string> can_yaml_vec = can_yaml["can_config_files"].as<std::vector<std::string>>();
	for (int i = 0; i < can_yaml_vec.size(); i++)
	{
		if (can_yaml["can_type"].as<std::string>() == "SocketCAN")
		{
			can_channel_vec_.push_back(std::make_shared<can::SocketCANChannel>(can_config_data_path +
																			   "can_tools_config/" + can_yaml_vec[i]));
		}
		else
		{
#ifdef ESD_DRIVERS
			can_channel_vec_.push_back(std::make_shared<can::EsdCanChannel>(can_config_data_path +
																			"can_tools_config/" + can_yaml_vec[i]));
			std::shared_ptr<can::BaseCanChannel> temp_ptr = can_channel_vec_[i];
			std::vector<unsigned int> ids_vec = dbc_analysis::DbcAnalysis::getInstance()->ids();
			dynamic_cast<can::EsdCanChannel *>(can_channel_vec_[i].get())->add_ids(ids_vec); //legal
#else
			std::cout << "\033[31m Only SocketCAN is supported. \033[0m" << std::endl;
			throw 1;
#endif
			can_channel_vec_[i]->register_asyn_read_cb(can_cb_fun);
			can_channel_vec_[i]->start_asyn_read();
		}
	}
	init_map();
}

void Can2RosAndCsvNode::init_map()
{
	auto can_msgs = dbc_analysis::DbcAnalysis::getInstance()->getMessages();
	for (std::map<long, Message>::const_iterator m = can_msgs.begin(); m != can_msgs.end(); ++m)
	{
		auto_msgs::can_msg temp_can_msg;
		temp_can_msg.can_msg_name = m->second.name;
		temp_can_msg.id = m->second.id;
		can_msgs_ros_map_.insert(std::pair<std::string, auto_msgs::can_msg>(m->second.name, temp_can_msg));
		//std::map<std::string, ros::Publisher> pub_map;
		//ros::Publisher localizition_pub_ = nh.advertise<auto_msgs::localizition>("localizition", 3);
		ros::Publisher temp_pub = nh.advertise<auto_msgs::can_msg>("/can/" + m->second.name, 15);
		pub_map_.insert(std::pair<std::string, ros::Publisher>(m->second.name, temp_pub));
		std::string out_file_name = "can_" + m->second.name + ".csv";
		if (PathExists(auto_ros_localization_path + "log_data/") == false)
		{
			CreateDir(auto_ros_localization_path + "log_data/");
		}
		std::string sub_path = auto_ros_localization_path + "log_data/" + getCurrentTimeStr() + "/";
		if (PathExists(sub_path) == false)
		{
			CreateDir(sub_path);
		}
		out_file_name = sub_path + out_file_name;
		std::shared_ptr<io::CSVWriter> temp_csv_ptr = std::make_shared<io::CSVWriter>(out_file_name);
		csv_map_.insert(std::pair<std::string, std::shared_ptr<io::CSVWriter>>(m->second.name, temp_csv_ptr));
		std::vector<std::string> head_string;
		head_string.push_back("time");
		for (int i = 0; i < m->second.signals.size(); i++)
		{
			head_string.push_back(m->second.signals[i].name);
		}
		temp_csv_ptr->write_row<std::string>(head_string);
	}
}
void Can2RosAndCsvNode::can_cb(can_frame frame)
{
	mtx_.lock();
	auto ros_time_now = ros::Time::now();
	double time_now = ros_time_now.toSec();
	auto iter = dbc_analysis::DbcAnalysis::getInstance()->getMessages().find(frame.can_id);
	if (iter == dbc_analysis::DbcAnalysis::getInstance()->getMessages().end())
	{
		mtx_.unlock();
		//std::cout << " iter == dbc_analysis::DbcAnalysis::getInstance()->getMessages().end()" << std::endl;
		return;
	}
	auto iter_can_ros_msg = can_msgs_ros_map_.find(iter->second.name);
	if (iter_can_ros_msg == can_msgs_ros_map_.end())
	{
		mtx_.unlock();
		//std::cout << "iter_can_ros_msg == can_msgs_ros_map_.end()" << std::endl;
		return;
	}

	auto iter_pub = pub_map_.find(iter->second.name);
	if (iter_pub == pub_map_.end())
	{
		mtx_.unlock();
		return;
	}
	auto iter_csv = csv_map_.find(iter->second.name);
	if (iter_csv == csv_map_.end())
	{
		mtx_.unlock();
		return;
	}
	//std::cout << " i am here" << std::endl;
	Message temp_message = iter->second;
	std::vector<double> unpack_sigs_vec(temp_message.signals.size());
	if (can_util::unpackCanmsg(temp_message, frame, unpack_sigs_vec) != PACK_UNPACK_SUCCESS)
	{
		mtx_.unlock();
		return;
	}

	//std::cout << "i am ok" << std::endl;
	iter_can_ros_msg->second.can_sigs.clear();
	std::vector<double> temp_vec;
	temp_vec.push_back(time_now);
	std::vector<auto_msgs::can_sig> temp_sig_vec;
	for (int i = 0; i < unpack_sigs_vec.size(); i++)
	{
		auto_msgs::can_sig temp_sig;
		temp_sig.sig_name = temp_message.signals[i].name;
		temp_sig.value = unpack_sigs_vec[i];
		temp_vec.push_back(unpack_sigs_vec[i]);
		temp_sig_vec.push_back(temp_sig);
	}
	iter_can_ros_msg->second.can_sigs = temp_sig_vec;
	iter_can_ros_msg->second.header.stamp = ros_time_now; // 时间戳
	iter_csv->second->write_row<double>(temp_vec);
	iter_pub->second.publish(iter_can_ros_msg->second);
	mtx_.unlock();
}
void Can2RosAndCsvNode::step()
{
	//ros::spinOnce(); //可用回调函数
	loop_rate_ptr_->sleep();
}

int main(int argc, char **argv)
{
	std::string can_node_yaml_file_name = can_config_data_path + "can_tools_config/can_node.yaml";
	ros::init(argc, argv, "can");
	ROS_INFO("INIT SUSCESS");
	Can2RosAndCsvNode node(can_node_yaml_file_name);
	ROS_INFO("NODE INIT");

	while (1)
	{
		node.step();
	}

	return 0;
}