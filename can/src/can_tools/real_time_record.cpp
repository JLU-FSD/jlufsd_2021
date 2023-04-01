#include "ros/ros.h"
#include "auto_msgs/localizition.h"
#include "common_tool/csv_writer.h"
#include "can_config_data_path.h"
#include "common_tool/angle_helper.h"
//回调函数，简单来说就是收到了这个消息，就做出什么反应在这里面定义。这里是打印一行东西
io::CSVWriter record_writer(can_config_data_path + "real_time_record.csv");

void localizition_cb(const auto_msgs::localizition &msg)
{
	record_writer.write_row<double>({msg.x, msg.y, 
                                    auto_ros::common_tool::angle_mins_180to180(msg.heading)});
	std::cout << "x:" << msg.x << "y:" << msg.y << "heading:" << msg.heading << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "record");
	ros::NodeHandle n;
	//订阅了“chatter”这个topic
	record_writer.write_row<std::string>({"x", "y", "theta"});
	ros::Subscriber sub = n.subscribe("localizition", 3, localizition_cb);
	ros::spin();
	return 0;
}