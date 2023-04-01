#pragma once
namespace auto_ros
{
namespace common_tool
{
double angle_mins_pi2pi(double angle_degree)
{
	int angle_int = std::floor(angle_degree / 360.0);
	angle_degree = angle_degree - angle_int * 360.0;
	if (angle_degree > 180)
	{
		angle_degree -= 360;
	}
	else if (angle_degree <= -180)
	{
		angle_degree += 360;
	}
	return angle_degree * M_PI / 180;
}
double angle_mins_180to180(double angle_degree)
{
	int angle_int = std::floor(angle_degree / 360.0);
	angle_degree = angle_degree - angle_int * 360.0;
	if (angle_degree > 180)
	{
		angle_degree -= 360;
	}
	else if (angle_degree <= -180)
	{
		angle_degree += 360;
	}
	return angle_degree;
}
} // namespace common_tool
} // namespace auto_ros