#include "planning_math_tool.h"



double planning::angleCorrection(const double theta)
{
    double theta_correct = theta;
    while(1)
    {
        // std::cout<< theta_correct <<std::endl;
        // std::cout<< (theta_correct > -M_PI)<<std::endl;
        // std::cout<< (theta_correct <= M_PI)<<std::endl;
        if(theta_correct > M_PI)
        {
            theta_correct -= 2 * M_PI;
        }

        if(theta_correct <= -M_PI)
        {//不存在-pi
            theta_correct += 2 * M_PI;
        }

        if(theta_correct > -M_PI && theta_correct <= M_PI)
        {
            break;
        }
    }
    // std::cout<< "11111" <<std::endl;
    return theta_correct;
}   

double planning::angleDifference(const double theta1, const double theta2)
{
    // //首先判断两个角度所处相限
    // theta1 = planning::angleCorrection(theta1);
    // theta2 = planning::angleCorrection(theta2);
    return planning::angleCorrection(theta1 - theta2);;
}

double planning::box(const double & num, const double & down_boudry, const double & up_boudry)
{
    if(down_boudry > up_boudry)
    {
        std::cout<<"up_boudry should bigger than down_boudry!"<<std::endl;
        return num;
    }

    if(num < down_boudry)
    {
        return down_boudry;
    }

    if(num > up_boudry)
    {
        return up_boudry;
    }

    return num;
}

double planning::linearInterpolation(const double & x, const double & down_x, const double & up_x, const double & down_value, const double & up_value)
{
    double k = (x - down_x) / (up_x - down_x);
    return down_value * (1 - k) + up_value * k;
}


double planning::norm2(double _num1, double _num2)
{
    return sqrt(pow(_num1, 2) + pow(_num2, 2));
}

double planning::curvature(double _vx, double _vy, double _ax, double _ay)
{
    return (_vx * _ax - _vy * _ay)/pow(planning::norm2(_vx, _vy),3);
}

double planning::triangleArea(const double & a, const double & b, const double & c)
{
    double area = 0;
    if(a+b>c && b+c>a && a+c>b)                          //判断三角形是否成立
    {
        double s = 1.0 / 2 * (a+b+c);                        /*此处很关键，刚开始打1/2，结果输什么数进去都是0。
                                                                细细斟酌后才发现int的1/2=0的，要打1.0/2才对。*/
        area = sqrt(s*(s-a)*(s-b)*(s-c));         //sqrt表示平方根，调用函数后才可使用
    }
    else
    {
        std::cout<<"该三边不构成三角形"<<std::endl;
        std::cout<<"a = "<<a<<" b = "<<b<<" c = "<<c<<std::endl;
    }
    return area;		
}

double planning::triangleCircumCurvature(const double & a, const double & b, const double & c)
{
    return  4 * planning::triangleArea(a, b, c)/(a * b * c);		
}
