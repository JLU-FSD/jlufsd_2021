#include "common_tool/clampPid.h"
#include <stdio.h>
#include <ros/ros.h>
clampPid::clampPid(double Kp_local, double Ki_local, double Kd_local, double deltaTime_local) : Kp(Kp_local), Ki(Ki_local), Kd(Kd_local), deltaTime(deltaTime_local)
{
	IntergrationDownLimit = std::numeric_limits<double>::min();
	IntergrationUplimit = std::numeric_limits<double>::max();
	downLimit = std::numeric_limits<double>::min();
	upLimit = std::numeric_limits<double>::max();
	Intergration = 0;
	output = 0;
}
clampPid::clampPid(double Kp_local, double Ki_local, double Kd_local,
				   double deltaTime_local,
				   double downLimit_local,
				   double upLimit_local,
				   double IntergrationDownLimit_local,
				   double IntergrationUplimit_local) : Kp(Kp_local), Ki(Ki_local), Kd(Kd_local),
													   deltaTime(deltaTime_local),
													   IntergrationDownLimit(IntergrationDownLimit_local),
													   IntergrationUplimit(IntergrationUplimit_local),
													   downLimit(downLimit_local),
													   upLimit(upLimit_local)
{
	Intergration = 0;
	output = 0;
}
void clampPid::resetIntergration()
{
	Intergration = 0;
}
double clampPid::step(double error)
{

	double tempI;
	tempI = Intergration + Ki * error * deltaTime;
	double temp = Kp * error + tempI;
	if ((tempI > IntergrationUplimit) && (error >= 0)) //|| (temp>=upLimit)
	{
		Intergration = IntergrationUplimit;
	}
	else if ((tempI < IntergrationDownLimit) && (error < 0))
	{
		Intergration = IntergrationDownLimit;
	}

	else
	{
		Intergration = tempI;
	}

	temp = Kp * error + Intergration;
	if (temp >= upLimit)
	{
		//ROS_INFO("outPut is limited to upLimit %f", upLimit);
		output = upLimit;
	}
	else if (temp <= downLimit)
	{
		output = downLimit;
		//ROS_INFO("outPut is limited to downLimit %f", downLimit);
	}
	else
	{
		//ROS_INFO("outPut is nnormal %f",temp);
		output = temp;
	}
	return output;
}
void clampPid::setKp(double Kp_local)
{
	Kp = Kp_local;
}
void clampPid::setKi(double Ki_local)
{
	Ki = Ki_local;
}
void clampPid::setDeltaTime(double deltaTime_local)
{

	deltaTime = deltaTime_local;
}
void clampPid::setAllParameter(double Kp_local, double Ki_local, double Kd_local,
							   double deltaTime_local,
							   double downLimit_local,
							   double upLimit_local,
							   double IntergrationDownLimit_local,
							   double IntergrationUplimit_local)
{
	Kp = Kp_local;
	Ki = Ki_local;
	Kd = Kd_local;
	deltaTime = deltaTime_local;
	downLimit = downLimit_local;
	upLimit = upLimit_local;
	IntergrationDownLimit = IntergrationDownLimit_local;
	IntergrationUplimit = IntergrationUplimit_local;
}
