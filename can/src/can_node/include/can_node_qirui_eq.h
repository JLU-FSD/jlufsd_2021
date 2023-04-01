#pragma once
#include "can_node_base.h"
#include "auto_msgs/localizition.h"
#include "auto_msgs/chassis.h"
#include "auto_msgs/control_cmd.h"
#include "can_pool.hpp"
#include "common_tool/coordinate_trans.h"
#include "geodetic_conv.hpp"
#include <common_tool/csv_writer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <thread> // std::thread
#include <mutex>  // std::mutex
std::map<int, std::string> m2_nav_flag_map = {
	{0x00, "initial"},
	{0x01, "coarse alignment"},
	{0x02, "fine alignment"},
	{0x03, "single antenna positioning"},
	{0x04, "dual antenna positioning"},
	{0x05, "single antenna differential"},
	{0x06, "odom combination"},
	{0x08, "pure inertial navigation"},
	{0x09, "zero speed correction"},
	{0x0b, "dual antenna differential"},
	{0x0c, "dynamic calibration"},
	{0x0f, "system abnormality"}};
std::map<int, std::string> m2_gps_flag_map = {
	{1, "data valid"},
	{3, "speed postion valid"},
	{7, "orientation valid"},
	{15, "diffirential valid"},
	{31, "time valid"},
	{63, "differential floating"},
	{127, "differential fixed"}};
/*#define command "/sbin/ip link set can0 type can bitrate 125000" //将CAN0波特率设置为125000 bps
#define up "echo \" \" | sudo -S ifconfig vcan0 up"				 //打开CAN0
#define down "echo \" \" | sudo -S ifconfig vcan0 down"			 //关闭CAN0*/
class CanNodeQiRuiEq : public CanNodeBase
{
public:
	CanNodeQiRuiEq() = delete;
	CanNodeQiRuiEq(std::string can_node_yaml_file_name);
	virtual void step();
	bool publish_localiziton();
	bool publish_chassis();
	void control_cmd_to_can(double throttle, double brake, double tire_angle, int gear);
	std::shared_ptr<io::CSVWriter> log_writer_ptr_;

private:
	YAML::Node chassis_yaml_;
	//std::unique_ptr<dbc_analysis::DbcAnalysis> dbc_ptr_;
	geodetic_converter::GeodeticConverter geo_converter_;
	ros::Publisher localizition_pub_ = nh_.advertise<auto_msgs::localizition>("localizition", 3);
	auto_msgs::localizition localizition_;
	ros::Publisher chassis_pub_ = nh_.advertise<auto_msgs::chassis>("chassis", 3);
	auto_msgs::chassis chassis_;
	//pub debug msg
	//ros::Publisher pure_pursuit_debug_pub =nh_.advertise
	//    <pure_pursuit_pkg::pure_pursuit_debug>("pure_pursuit_debug", 3);
	//subscribe apollo wrapper topic
	//ros::Rate loop_rate_;
	//ros::Rate loop_rate_;
	auto_msgs::control_cmd control_cmd_;
	watchDog control_cmd_dog_;
	Eigen::Vector3d cg_in_imu_frame_;

	ros::Subscriber control_sub_ =
		nh_.subscribe("control_cmd", 3, &CanNodeQiRuiEq::cb_control_cmd, this);
	void cb_control_cmd(const auto_msgs::control_cmd &msg);
	void no_food();
	bool check_dog();
	void hard_brake_to_can();
	void speed_to_m2(double speed_kmh, int gear);
};