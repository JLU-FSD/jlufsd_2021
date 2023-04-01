#pragma once
namespace auto_ros
{
namespace common_tool
{
template <class T>
T sign(T input)
{
	if (input > 0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}
} // namespace common_tool
} // namespace auto_ros