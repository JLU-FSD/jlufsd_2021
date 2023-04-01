#include "socket_can_node.h"
#include "ros/ros.h"
#include "can_config_data_path.h"
#include <yaml-cpp/yaml.h>
#include <common_tool/csv_writer.h>
int main(int argc, char **argv)
{
	std::string can_node_yaml_file_name = can_config_data_path + "can_node.yaml";

	YAML::Node can_yaml = YAML::LoadFile(can_node_yaml_file_name); //也可以这样读取文件
	std::string log_file_name = can_yaml["log_file_name"].as<std::string>();
	std::string can_frame_file_name = can_config_data_path + "test_mutiple_step.yaml";
	YAML::Node cmd_yaml = YAML::LoadFile(can_frame_file_name); //也可以这样读取文件
	ros::init(argc, argv, "osqp_control");
	ROS_INFO("INIT SUSCESS");
	CanNode socket_can_node(can_node_yaml_file_name);
	double time = 0;
	int step = 0;
	double throttle;
	double brake;
	double tire_angle;
	double gear;
	ROS_INFO("NODE INIT");
	io::CSVWriter log_writer(log_file_name);
	std::vector<std::string> header = {"steer_cmd", "steer_fb"};
	log_writer.write_row<std::string>(header);
	/*can_frame frame = {0};
	frame.can_id = 0x706;
	frame.can_dlc = 8;
	frame.data[0] = 0x15;
	frame.data[1] = 0x68;
	frame.data[2] = 0x1F;
	frame.data[3] = 0xFF;
	frame.data[4] = 0x34;
	frame.data[5] = 0xFF;
	frame.data[6] = 0x58;
	frame.data[7] = 0xE0;
	std::vector<double> unpack_sigs(3);
	auto iter = dbc_analysis::DbcAnalysis::getInstance()->getMessages().find(frame.can_id);
	Message temp_message = iter->second;
	can_util::unpackCanmsg(temp_message, frame, unpack_sigs);
	can_frame frame_pack = {0};
	//packCanmsg(const Message &m, const std::vector<double> &signals_value, can_frame &msg)
	can_util::packCanmsg(temp_message, unpack_sigs, frame_pack);
	std::cout << "yaw:" << unpack_sigs[0] << "	pitch:" << unpack_sigs[1]
			  << "	roll:" << unpack_sigs[2] << std::endl;
	unsigned int temp0 = frame_pack.data[0];
	unsigned int temp1 = frame_pack.data[1];
	unsigned int temp2 = frame_pack.data[2];
	unsigned int temp3 = frame_pack.data[3];

	unsigned int temp4 = frame_pack.data[4];
	unsigned int temp5 = frame_pack.data[5];
	unsigned int temp6 = frame_pack.data[6];
	unsigned int temp7 = frame_pack.data[7];
	std::cout << std::hex << temp0 << std::endl;
	std::cout << std::hex << temp0 << " " << temp1 << " " << temp2 << " " << temp3 << " "
			  << temp4 << " " << temp5 << " " << temp6 << " " << temp7 << " " << std::endl;*/
	std::vector<double> unpack_sigs(4);
	can_frame frame = {0};
	frame.can_id = 0x705;
	unpack_sigs[0] = 0.0;
	unpack_sigs[1] = -1.0;
	unpack_sigs[2] = -1.0;
	unpack_sigs[3] = 1.0;
	auto iter = dbc_analysis::DbcAnalysis::getInstance()->getMessages().find(frame.can_id);
	Message temp_message = iter->second;
	can_util::packCanmsg(temp_message, unpack_sigs, frame);
	unsigned int temp0 = frame.data[0];
	unsigned int temp1 = frame.data[1];
	unsigned int temp2 = frame.data[2];
	unsigned int temp3 = frame.data[3];

	unsigned int temp4 = frame.data[4];
	unsigned int temp5 = frame.data[5];
	unsigned int temp6 = frame.data[6];
	unsigned int temp7 = frame.data[7];
	std::cout << std::hex << temp0 << std::endl;
	std::cout << std::hex << temp0 << " " << temp1 << " " << temp2 << " " << temp3 << " "
			  << temp4 << " " << temp5 << " " << temp6 << " " << temp7 << " " << std::endl;
	std::vector<double> unpack_sigs2(4);
	can_util::unpackCanmsg(temp_message, frame, unpack_sigs2);
	std::cout << unpack_sigs2[0] << " :" << unpack_sigs2[1] << " :" << unpack_sigs2[2] << " :" << unpack_sigs2[3] << std::endl;

	while (1)
	{

		return 0;
		time += 1.0 / socket_can_node.rate_;
		//socket_can_node.publish_localiziton();
		//socket_can_node.publish_chassis();
		//speed_to_m2(10 * 3.6, 0);
		//localizition_pub_.publish(localizition_);
		//chassis_pub_.publish(chassis_);

		if (step < cmd_yaml.size())
		{

			if (time < cmd_yaml[step]["time"].as<double>())
			{
				throttle = cmd_yaml[step]["cmd"][0].as<double>();
				brake = cmd_yaml[step]["cmd"][1].as<double>();
				tire_angle = cmd_yaml[step]["cmd"][2].as<double>();
				gear = cmd_yaml[step]["cmd"][3].as<double>();
			}
			else
			{

				throttle = cmd_yaml[step]["cmd"][0].as<double>();
				brake = cmd_yaml[step]["cmd"][1].as<double>();
				tire_angle = cmd_yaml[step]["cmd"][2].as<double>();
				gear = cmd_yaml[step]["cmd"][3].as<double>();
				step += 1;
				time = 0;
			}
		}
		else
		{
			throttle = 0;
			brake = 0;
			tire_angle = 0;
			gear = 0;
		}
		//std::cout << "throttle is:" << throttle << "	brake is:" << brake << "	tire angle is:"
		//		  << tire_angle << "	gear is:" << gear << "	time is:" << time << "	step is:" << step << std::endl;
		//bool read_can_pool_sig(const unsigned int can_id, const std::string sig_name, double &sig_value);
		double steer_fb = 0;
		if (socket_can_node.read_can_pool_sig(0x394, "SteerAngle_Ret", steer_fb) == false)
		{
			std::cout << "test_chasiss steer fb failed \033[0m" << std::endl;
		}
		std::vector<double> temp_vec = {tire_angle * 16.2, steer_fb};
		std::cout << "cmd and steer_fb: " << temp_vec[0] << "  " << temp_vec[1] << std::endl;
		log_writer.write_row<double>(temp_vec);
		//void CanNode::control_cmd_to_can(double throttle, double brake, double tire_angle, int grear)
		//socket_can_node.control_cmd_to_can(throttle, brake, tire_angle, gear);

		ros::spinOnce(); //可用回调函数
		socket_can_node.publish_localiziton();
		socket_can_node.publish_chassis();
		socket_can_node.loop_rate_ptr_->sleep();
	}
	return 0;
}
