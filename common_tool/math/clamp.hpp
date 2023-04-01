#pragma once
namespace auto_ros
{
namespace common_tool
{
template <class T>
T clamp(T input, T down_limit, T up_limit)
{
	if (input < down_limit)
	{
		return -down_limit;
	}
	else if (input > up_limit)
	{
		return up_limit;
	}
	else
	{
		return input;
	}
}
} // namespace common_tool
} // namespace auto_ros