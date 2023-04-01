#pragma once
namespace auto_ros
{
namespace common_tool
{
template <class T>
T eliminate_range(T input, T down_limit, T medium_value, T up_limit)
{
	if (input < up_limit && input > down_limit)
	{
		if (input >= medium_value)
			return up_limit;
		else
		{
			return down_limit;
		}
	}
	else
	{
		return input;
	}
}
} // namespace common_tool
} // namespace auto_ros