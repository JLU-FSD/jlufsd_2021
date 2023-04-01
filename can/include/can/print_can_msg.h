#pragma once
#include <linux/can.h>	   //use linux can
#include <linux/can/raw.h> //use linux can
#include <iostream>
void msgPrint(const can_frame &msg)
{
	std::cout << "id is:" << msg.can_id << std::endl;
	std::cout << "dlc is:" << msg.can_dlc << std::endl;
	for (int i = 0; i < 8; i++)
	{
		printf("%x ", msg.data[i]);
	}
	printf("\n");
}