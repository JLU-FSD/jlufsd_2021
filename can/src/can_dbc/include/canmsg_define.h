#ifndef _CANMSG_DEFINE_H_
#define _CANMSG_DEFINE_H_

#include <vector>
#include <map>
#include <algorithm>
#include <functional>
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long int64_T;
typedef unsigned long uint64_T;
typedef float real32_T;
typedef double real64_T;

struct Signal
{
	std::string name;
	int startBit;
	int length;
	double factor;
	double offset;
	double maximum;
	double minimum;
	int dataType;
	int is_unsigned;
	std::string unit;

	bool operator<(const Signal &s) const // 升序排序时必须写的函数
	{
		return startBit < s.startBit;
	}
	bool operator>(const Signal &s) const // 降序排序时必须写的函数
	{
		return startBit > s.startBit;
	}
};

struct Message
{
	long id;
	std::string name;
	int length;
	std::vector<Signal> signals;

	bool operator<(const Message &msg) const // 升序排序时必须写的函数
	{
		return id < msg.id;
	}
	bool operator>(const Message &msg) const // 降序排序时必须写的函数
	{
		return id > msg.id;
	}
};

struct Canmsg
{
	long id;
	int length;
	uint8_T data[8];
};

enum PackUnpackStat
{
	SIGNAL_SIZE_MISMATCH = -3,
	UNPACK_ID_MISMATCH = -2,
	UNPACK_LENGTH_MISMATCH = -1,
	PACK_UNPACK_SUCCESS = 0
};

#endif //_CANMSG_DEFINE_H_
