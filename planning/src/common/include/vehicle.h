#ifndef VEHICLE_H
#define VEHICLE_H

#include "ros/ros.h"

namespace planning
{
    struct Vehicle
    {
        double                          x;
        double                          y;
        double                          yaw;
        double                          steer;//当前转向角

        double                          vx;//纵向速度，即仪表盘速度
        double                          vy;//侧向速度
        double                          r;//横摆角速度

        Vehicle()
        {
            x = 0;
            y = 0;
            yaw = 0;
            
            vx = 0;
            vy = 0;
            r = 0;
        }
    };

    struct VehicleParam
    {
        //惯性参数
        double                          m;
        double                          I;//转动惯量
        //几何参数
        double                          a;
        double                          b;
        double                          l;
        //侧偏刚度
        double                          Cf;//前轮侧便刚度，单位nm/弧度
        double                          Cr;//后轮侧便刚度，单位nm/弧度
        //阻力系数
        double                          k_air;//空气动力学阻力系数----Cd * A * 3.6 * 3.6 / 21.15  

        VehicleParam()
        {
            m = 2080;
            I = 1500;
            a = 2.1;
            b = 1.85;
            l = a + b;

            Cf = 80000;
            Cr = 80000;

            k_air = 0.696 * 3.6 * 3.6 / 21.15;
        }
    };

    double distance(const Vehicle & v1, const Vehicle & v2);

} // namespace planning

#endif //VEHICLE_H