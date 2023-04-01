#include "ros/ros.h"
#include "auto_msgs/localizition.h"
#include "can_node.h"
#include <fstream>
#include "can_config_data_path.h"
#include <yaml-cpp/yaml.h>
int main(int argc, char **argv)
{
	std::string can_node_yaml_file_name = can_config_data_path + "can_node.yaml";
	ros::init(argc, argv, "can");
	ROS_INFO("INIT SUSCESS");
	CanNode can_node(can_node_yaml_file_name);
	double latitude = 0;
	double longtitude = 0;
	double altitude = 0;
	ROS_INFO("NODE INIT");
    sleep(3.0);
	if (can_node.read_can_pool_sig(0x703, "Latitude", latitude) == false)
	{
		std::cout << "test_chasiss steer fb failed \033[0m" << std::endl;
	}
	if (can_node.read_can_pool_sig(0x703, "Longitude", longtitude) == false)
	{
		std::cout << "test_chasiss steer fb failed \033[0m" << std::endl;
	}
	if (can_node.read_can_pool_sig(0x704, "Altitude", altitude) == false)
	{
		std::cout << "test_chasiss steer fb failed \033[0m" << std::endl;
	}
	std::cout << "lla is:" << std::endl
			  << latitude << "," << longtitude << "," << altitude << std::endl;
	std::fstream fout;
	fout.open(can_config_data_path + "navi.yaml", std::ios::out | std::ios::trunc);
	if (fout.is_open())
	{
		fout << std::setprecision(12) << "latitude: " << latitude << std::endl;
		fout << std::setprecision(12) << "longitude: " << longtitude << std::endl;
		fout << std::setprecision(12) << "altitude: " << altitude << std::endl;
		fout << "cg_in_imu: [0.0,0.0,0.0]" << std::endl;
	}
	return 0;
}