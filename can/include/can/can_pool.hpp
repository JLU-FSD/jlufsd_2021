#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include "dbc_file_analysis.h"
#include "common_tool/watchDog.h"
class CanSigsPool
{
public:
	unsigned int can_id_;
	std::map<std::string, double> sigs_; //serch by signal name
};
class CanMsgsPool
{
public:
	std::map<unsigned int, watchDog> dog_map_;
	std::map<unsigned int, std::mutex> mutex_map_;
	std::map<unsigned int, CanSigsPool> msgs_; //can_id is the key
	void init(std::map<long, Message> &messages)
	{
		for (auto can_msg_iter = messages.begin(); can_msg_iter != messages.end(); can_msg_iter++)
		{
			CanSigsPool can_sigs_pool;
			can_sigs_pool.can_id_ = can_msg_iter->first;
			watchDog temp_dog(0, 0, 0, 50, 100);
			dog_map_[can_msg_iter->first] = temp_dog;
			for (int sig_index = 0; sig_index < can_msg_iter->second.signals.size(); sig_index++)
			{
				can_sigs_pool.sigs_[can_msg_iter->second.signals[sig_index].name] = 0;
			}
			//mp.insert(make_pair<int,string>(2,"bbbbb"));
			//std::cout << "msg name is:" << can_msg_iter->second.name << std::endl;
			unsigned int can_id = can_msg_iter->first;
			msgs_[can_id] = can_sigs_pool;
		}
	}
	void set_all_dog(int foodPerTurn, int hungerPerTurn, int hungerMin,
					 int criticalPoint, int hungerMax)
	{
		for (auto iter = dog_map_.begin(); iter != dog_map_.end(); iter++)
		{
			iter->second.setAllParameter(foodPerTurn, hungerPerTurn, hungerMin,
										 criticalPoint, hungerMax);
		}
	}
	void set_one_dog(unsigned int can_id, int foodPerTurn, int hungerPerTurn, int hungerMin,
					 int criticalPoint, int hungerMax)
	{
		auto iter = dog_map_.find(can_id);
		if (iter == dog_map_.end())
		{
			std::cout << "\033[31m CanMsgsPool::set_one_dog failed,can id not in dbc \033[0m" << std::endl;
		}
		else
		{
			iter->second.setAllParameter(foodPerTurn, hungerPerTurn, hungerMin,
										 criticalPoint, hungerMax);
		}
	}
	bool read_frame_sigs(unsigned int can_id, std::map<std::string, double> &can_sigs, bool hunger = false)
	{

		auto msg_iter = msgs_.find(can_id);
		if (msg_iter == msgs_.end())
		{
			std::cout << "\033[31m CanMsgsPool::read_signal failed,can id 0x " << std::hex << can_id << " not in dbc \033[0m" << std::endl;
			return false;
		}
		mutex_map_[can_id].lock();
		if (hunger == true)
		{
			dog_map_[can_id].noFood();
		}
		if (dog_map_[can_id].isDied())
		{
			std::cout << "\033[31m CanMsgsPool::read_signal failed \033[0m" << std::hex << can_id
					  << "\033[31m time out \033[0m" << std::dec << std::endl;
			std::cout << "\033[31m CanMsgsPool::read_signal failed \033[0m"
					  << std::hex << can_id << std::endl;

			mutex_map_[can_id].unlock();
			return false;
		}
		can_sigs = msg_iter->second.sigs_;
		mutex_map_[can_id].unlock();
		return true;
	}
	bool read_signal(unsigned int can_id, std::string sig_name, double &sig_value, bool hunger = false)
	{
		auto msg_iter = msgs_.find(can_id);
		if (msg_iter == msgs_.end())
		{
			std::cout << "\033[31m CanMsgsPool::read_signal failed,can id not in dbc \033[0m" << std::endl;
			return false;
		}
		auto sig_iter = msg_iter->second.sigs_.find(sig_name);
		if (sig_iter == msg_iter->second.sigs_.end())
		{
			std::cout << "\033[31m CanMsgsPool::read_signal failed,sig_name not in dbc \033[0m" << std::endl;
			return false;
		}
		mutex_map_[can_id].lock();
		if (hunger == true)
		{
			dog_map_[can_id].noFood();
		}
		std::cout << dog_map_[can_id].getHunger() << std::endl;
		if (dog_map_[can_id].isDied())
		{
			std::cout << "\033[31m CanMsgsPool::read_signal failed \033[0m" << std::hex << can_id << ":"
					  << sig_name << "\033[31m time out \033[0m" << std::dec << std::endl;
			mutex_map_[can_id].unlock();
			return false;
		}
		sig_value = sig_iter->second;
		mutex_map_[can_id].unlock();
		return true;
	}
	void write_pool(can_frame frame)
	{
		auto iter = dbc_analysis::DbcAnalysis::getInstance()->getMessages().find(frame.can_id);
		if (iter == dbc_analysis::DbcAnalysis::getInstance()->getMessages().end())
		{

			return;
		}

		Message temp_message = iter->second;
		mutex_map_[frame.can_id].lock();
		dog_map_[frame.can_id].giveFood();

		std::vector<double> unpack_sigs(temp_message.signals.size());
		if (can_util::unpackCanmsg(temp_message, frame, unpack_sigs) != PACK_UNPACK_SUCCESS)
		{
			std::cout << "\033[31m write_pool inner can_util::unpackCanmsg failed\033[0m" << std::endl;
		}

		for (int i = 0; i < unpack_sigs.size(); i++)
		{

			msgs_[frame.can_id].sigs_[temp_message.signals[i].name] = unpack_sigs[i];
		}
		mutex_map_[frame.can_id].unlock();
	}
};
