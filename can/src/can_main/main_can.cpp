#include "can_node_base.h"
#include "ros/ros.h"
#include "can_config_data_path.h"
#include <yaml-cpp/yaml.h>
#include "can_node_factory.hpp"
using namespace auto_ros::can;
int main(int argc, char **argv)
{
    
	std::string can_node_yaml_file_name = can_config_data_path + "can_node.yaml";
    std::string can_main_yaml_file_name = can_config_data_path + "can_main.yaml";
    YAML::Node can_main_yaml=YAML::LoadFile(can_main_yaml_file_name);     
    
	ros::init(argc, argv, "can");
	ROS_INFO("INIT SUSCESS");
    CanNodeFactory can_factory;
    std::shared_ptr<CanNodeBase> 
    can_node_ptr = can_factory.CreateSharedObject(can_main_yaml["can_node_class_name"].as<std::string>(),can_node_yaml_file_name);
	
	ROS_INFO("NODE INIT");
	while (1)
	{
		can_node_ptr->step();
	}

	return 0;
}