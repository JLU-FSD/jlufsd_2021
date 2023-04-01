#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include "socket_can_channel.hpp"
#define command "/sbin/ip link set can0 type can bitrate 125000" //将CAN0波特率设置为125000 bps
#define up "echo \" \" | sudo -S ifconfig vcan0 up"				 //打开CAN0
#define down "echo \" \" | sudo -S ifconfig vcan0 down"			 //关闭CAN0
void test_fun(can_frame &frame)
{
	std::cout << "test call back" << std::endl;
	std::cout << std::hex << frame.can_id << std::dec << std::endl;
}
int main()
{
	//system(down);
	//system(command);
	//system(up); //上面三行关闭CAN设备，设置波特率后，重新打开CAN设备
	std::string yaml_path_name = can_config_data_path + "socket_can0.yaml";
	can::SocketCANChannel test_can(yaml_path_name);
	test_can.start_asyn_read();
	test_can.register_asyn_read_cb(test_fun);

	can_frame frame = {0};
	int i = 0;
	while (1)
	{
		frame.can_id = 0x16;
		frame.can_dlc = 2;
		frame.data[0] = 0x01 + i;
		frame.data[1] = 0x02;
		frame.data[7]++;
		test_can.write_frame(frame);
		i++;
		sleep(0.01);
	}

	return 0;
}