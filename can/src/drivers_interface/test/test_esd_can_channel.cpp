#include "esd_can_channel.hpp"
#include "ros/ros.h"
#include "can_config_data_path.h"
#include <yaml-cpp/yaml.h>
#include <common_tool/csv_writer.h>
using namespace can;
int main(int argc, char **argv)
{
    std::string esd_yaml_path_name = can_config_data_path + "esd_can0.yaml";
    EsdCanChannel test_esd_can(esd_yaml_path_name);
    std::vector<unsigned int> ids_vec = {0x710, 0x18f01d48};
    test_esd_can.add_ids(ids_vec);
    test_esd_can.start_asyn_read();
    ros::init(argc, argv, "test_esd_can");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); //发布频率为10FPS
    can_frame test_frame = {};
    test_frame.can_id = 0x102;
    test_frame.can_dlc = 3;
    test_frame.data[0] = 0x12;
    test_frame.data[1] = 0x13;
    test_frame.data[2] = 0x14;
    while (n.ok())
    {
        test_esd_can.write_frame(test_frame);
        ros::spinOnce();   //可用回调函数
        loop_rate.sleep(); //休眠一段时间以使得发布频率为 10Hz。
    }
}