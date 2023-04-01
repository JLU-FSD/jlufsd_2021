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
#include "auto_ros_time.h"
using namespace auto_ros::common_tool;

int main(int argc, char **argv)
{
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
}