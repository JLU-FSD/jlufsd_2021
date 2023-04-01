#include "can_node_qirui_eq.h"
#include "common_tool/angle_helper.h"
#include "can_config_data_path.h"

CanNodeQiRuiEq::CanNodeQiRuiEq(std::string can_node_yaml_file_name) : CanNodeBase(can_node_yaml_file_name)
{
	std::string chassis_yaml_file_name = can_config_data_path + "chassis_yaml/" + "qirui_eq_chassis.yaml";
	chassis_yaml_ = YAML::LoadFile(chassis_yaml_file_name);
	std::cout << "i am ok" << std::endl;
}

void CanNodeQiRuiEq::cb_control_cmd(const auto_msgs::control_cmd &msg)
{
	control_cmd_ = msg;
	control_cmd_dog_.giveFood();
}
void CanNodeQiRuiEq::no_food()
{
	control_cmd_dog_.noFood();
}
bool CanNodeQiRuiEq::check_dog()
{
	if (control_cmd_dog_.isDied())
	{
		ROS_ERROR("control cmd is time out");
		return false;
	}
	else
	{
		return true;
	}
}
void CanNodeQiRuiEq::hard_brake_to_can()
{
	control_cmd_to_can(0.0, 0.0, 0.0, 0);
}
void CanNodeQiRuiEq::control_cmd_to_can(double throttle, double brake, double tire_angle, int gear)
{
	std::vector<double> control_can(6, 0);
	control_can[0] = gear;
	control_can[1] = 1;															//enable
	control_can[2] = 5 * brake;													//c_break mPa
	control_can[3] = 100 * throttle;											//c_throtle %
	control_can[4] = (tire_angle)*chassis_yaml_["steering_ratio"].as<double>(); //steer deg
	control_can[5] = 360;
	can_frame frame_0x020 = {0};
	//int packCanmsg(const Message &m, const std::vector<double> &signals_value, can_frame &msg);
	can_util::packCanmsg(dbc_analysis::DbcAnalysis::getInstance()->getMessages()[0x020], control_can, frame_0x020);
	can_ptr_vec_[can_yaml_["can_send_channel"].as<int>()]->write_frame(frame_0x020);
}
void CanNodeQiRuiEq::speed_to_m2(double speed_kmh, int gear)
{
}

bool CanNodeQiRuiEq::publish_localiziton()
{
	bool temp_flag = true;
	std::map<std::string, double> combine_status_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x710, combine_status_can_sigs, true) == false)
	{

		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, combine_status_can_sigs feedback failed\033[0m" << std::endl;
		//return false;
	}
	std::cout << "nav system status is:" << m2_nav_flag_map[int(combine_status_can_sigs["NavFlag"])] << std::endl;
	//std::cout << "gps status is:" << m2_gps_flag_map[int(combine_status_can_sigs["GpsFlag"])] << std::endl;
	std::map<std::string, double> pose_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x706, pose_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, pose feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> ll_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x703, ll_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, latitude and logtitude feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> altitude_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x704, altitude_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, altitude feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> z_acc_x_rate_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x722, z_acc_x_rate_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, Gro feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> xy_rate_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x723, xy_rate_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, Gro feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> gps_speed_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x705, gps_speed_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_localiziton failed, gps_speed_can_sigs feedback failed\033[0m"
				  << std::endl;
	}
	if (temp_flag == false)
	{
		return false;
	}
	std::cout << "lla is:" << std::setprecision(12) << ll_can_sigs["Latitude"] << "  " << ll_can_sigs["Longitude"] << "  "
			  << altitude_can_sigs["Altitude"] << std::endl;
	std::cout << "pose is:" << pose_can_sigs["Yaw"] << "  " << pose_can_sigs["Pitch"] << "  "
			  << pose_can_sigs["Roll"] << std::endl;
	Eigen::Vector3d av_vec; //angular velocity and matirx in imu
	av_vec << z_acc_x_rate_can_sigs["Gyro_X"] / 180 * M_PI, xy_rate_can_sigs["Gyro_Y"] / 180 * M_PI,
		xy_rate_can_sigs["Gyro_Z"] / 180 * M_PI;
	std::cout << "angular velocity in sensor frame is:" << av_vec.transpose() << std::endl;

	Eigen::Matrix3d av_matrix_imu;
	av_matrix_imu << 0, -av_vec(2), av_vec(1),
		av_vec(2), 0, -av_vec(0),
		-av_vec(1), av_vec(0), 0;

	Eigen::Vector3d imu_speed_vec_enu;
	imu_speed_vec_enu << gps_speed_can_sigs["VelocityEast"], gps_speed_can_sigs["VelocityNorth"],
		gps_speed_can_sigs["VelocityUp"];

	// frame imu car enu
	Eigen::Matrix3d imu_enu_mat;
	imu_enu_mat = Eigen::AngleAxisd(pose_can_sigs["Yaw"] / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(pose_can_sigs["Pitch"] / 180 * M_PI, Eigen::Vector3d::UnitX()) *
				  Eigen::AngleAxisd(pose_can_sigs["Roll"] / 180 * M_PI, Eigen::Vector3d::UnitY());

	Eigen::Matrix3d imu_car_mat;
	imu_car_mat = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
				  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
	Eigen::Matrix3d enu_car_mat;
	enu_car_mat = imu_enu_mat.transpose() * imu_car_mat;
	//end frame
	//cala speed in car frame
	Eigen::Matrix3d imu_enu_mat_change_rate = imu_enu_mat.transpose() * av_matrix_imu;
	Eigen::Vector3d cg_speed_enu = imu_speed_vec_enu + imu_enu_mat_change_rate * cg_in_imu_frame_;
	Eigen::Vector3d cg_speed_car = enu_car_mat.transpose() * cg_speed_enu;
	std::cout << "cg speed in car frame is:" << cg_speed_car.transpose() << std::endl;
	geo_converter_.geodetic2Enu(ll_can_sigs["Latitude"], ll_can_sigs["Longitude"],
								altitude_can_sigs["Altitude"], &localizition_.x, &localizition_.y, &localizition_.z);
	Eigen::Vector3d imu_position;
	imu_position << localizition_.x, localizition_.y, localizition_.z;
	std::cout << "imu position is:" << imu_position.transpose() << std::endl;
	Eigen::Vector3d cg_position;
	cg_position = imu_position + imu_enu_mat.transpose() * cg_in_imu_frame_;
	std::cout << "cg_position is:" << cg_position.transpose() << std::endl;
	localizition_.x = cg_position(0);
	localizition_.y = cg_position(1);
	localizition_.z = cg_position(2);

	localizition_.avx = z_acc_x_rate_can_sigs["Gyro_X"];
	localizition_.avy = xy_rate_can_sigs["Gyro_Y"];
	localizition_.avz = xy_rate_can_sigs["Gyro_Z"];
	localizition_.heading = auto_ros::common_tool::angle_mins_180to180(-pose_can_sigs["Yaw"] + 360 + 90);
	double heading_dag = localizition_.heading / 180 * M_PI;
	double vx = gps_speed_can_sigs["VelocityEast"] * std::cos(heading_dag) +
				gps_speed_can_sigs["VelocityNorth"] * std::sin(heading_dag);
	double vy = -gps_speed_can_sigs["VelocityEast"] * std::sin(heading_dag) +
				gps_speed_can_sigs["VelocityNorth"] * std::cos(heading_dag);
	localizition_.ve = gps_speed_can_sigs["VelocityEast"]; //cg_speed_car(0);
	localizition_.vn = gps_speed_can_sigs["VelocityNorth"];
	localizition_.vx = vx; //cg_speed_car(0);
	localizition_.vy = vy; //cg_speed_car(1);
	localizition_.ax = 0;
	localizition_.ay = 0;
	std::cout << "vx is:" << cg_speed_car(0) << " or:" << vx << std::endl;
	std::cout << "vy is:" << cg_speed_car(1) << " or:" << vy << std::endl;
	localizition_.vz = gps_speed_can_sigs["VelocityHorizon"]; //std::pow(std::pow(imu_speed_vec_enu(0), 2) + std::pow(imu_speed_vec_enu(1), 2), 0.5);
	//localizition_.nav_flag = combine_status_can_sigs["NavFlag"];
	//localizition_.gps_flag = combine_status_can_sigs["GpsFlag"];
	localizition_pub_.publish(localizition_);
}
bool CanNodeQiRuiEq::publish_chassis()
{
	//bool read_frame_sigs(unsigned int can_id, std::map<std::string, double> &can_sigs, bool hunger = false)
	bool temp_flag = true;
	std::map<std::string, double> steer_can_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x18f01d48, steer_can_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_chassis failed, steering feedback failed\033[0m" << std::endl;
	}
	std::map<std::string, double> speed_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x403, speed_sigs, true) == false)
	{
		temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_chassis failed, speed feedback failed\033[0m" << std::endl;
	}

	std::map<std::string, double> driving_mode_sigs;
	if (can_msgs_pool_.read_frame_sigs(0x036, driving_mode_sigs, true) == false)
	{
		//temp_flag = false;
		std::cout << "\033[31m CanNodeQiRuiEq::publish_chassis failed, driving_mode_sigs feedback failed\033[0m" << std::endl;
	}

	if (temp_flag == false)
	{
		return false;
	}
	//chassis_.steering_angle_f = steer_can_sigs["SteerAngle_Ret"] / chassis_yaml_["steering_ratio"].as<double>();

	chassis_.steering_angle_f = (steer_can_sigs["SAS_SteeringAngle"] - 4.432) / chassis_yaml_["steering_ratio"].as<double>();
	chassis_.speed_mps = speed_sigs["VehicleSpeed_Ret"] / 3.6; //VehicleSpeed_Ret
	chassis_.driving_mode = driving_mode_sigs["SysMode_Ret"];
	std::cout << "steering_angle_f:" << chassis_.steering_angle_f << std::endl;
	std::cout << "speed_mps:" << chassis_.speed_mps << std::endl;
	std::cout << "driving_mode(1:auto,else:manual):" << chassis_.driving_mode << std::endl;
	chassis_pub_.publish(chassis_);
	return true;
}
void CanNodeQiRuiEq::step()
{
	/*no_food();
	publish_localiziton();
	publish_chassis();
	if (control_cmd_dog_.isDied())
	{
		std::cout << "\033[31m CanNodeQiRuiEq::control_cmd_dog_.isDied() time out, \033[0m" << std::endl;
	}
	else
	{
		//control_cmd_to_can(0.0, 0, -10.0, 2);
		std::cout << "\033[31m control_to_cmd, \033[0m" << std::endl;
		control_cmd_to_can(control_cmd_.throttle, control_cmd_.brake, -control_cmd_.tire_angle_target_f, 2);
	}*/

	ros::spinOnce(); //可用回调函数
	loop_rate_ptr_->sleep();
}