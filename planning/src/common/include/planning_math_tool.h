#ifndef PLANNING_MATH_TOOL_H
#define PLANNING_MATH_TOOL_H

#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 
#include <math.h>

namespace planning
{
double angleCorrection(const double theta);
double angleDifference(const double theta1, const double theta2);

double box(const double & num, const double & down_boudry, const double & up_boudry);
//线性插值
double linearInterpolation(const double & x, const double & down_x, const double & up_x, const double & down_value, const double & up_value);
//二范数
double norm2(double _num1, double _num2);
double curvature(double _vx, double _vy, double _ax, double _ay);

double triangleArea(const double & a, const double & b, const double & c);//三角形面积
double triangleCircumCurvature(const double & a, const double & b, const double & c);//三角形外接圆曲率

}
#endif //PLANNING_MATH_TOOL_H

