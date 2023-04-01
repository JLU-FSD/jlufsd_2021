#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <algorithm>//min_element methord
#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 

#include <math.h> 
#include "planning_math_tool.h"

namespace planning
{
    struct PathPoint
    {
        double                          x;//同时也作为sl坐标系统的s
        double                          y;//同时也作为sl坐标系统的l
        double                          yaw;
        double                          cur;
        double                          s;//路程
        
        double                          left_width;
        double                          right_width;
        
        double                          vx;//纵向速度
        double                          ax;//纵向加速度
        double                          t;//纵向加速度

        double                          slope_angle;//道路坡度，弧度


        PathPoint()
        {
            x = 0;
            y = 0;
            yaw = 0;
            cur = 0;
            left_width = 0;
            right_width = 0;
            vx = 0;//纵向速度
            s = 0;//路程
            ax = 0;//纵向加速度
            t = 0;
            slope_angle = 0;
        }
    };

    struct Path
    {
        std::vector<PathPoint>          path;
        double                          precision;

        bool                            has_s;
        bool                            has_yaw;
        bool                            has_cur;
        bool                            has_v;
        bool                            has_t;
        bool                            has_width;
        bool                            has_a;

        void showInCsv(const char* _filename);
        void calculateS();
        void calculateYaw();//目前采用的是弧度制
        void calculateCur();
        void calculateCur2(const int & offset);//采用三点构成的外接圆半径的倒数为曲率
        void calculateT();
        void calculateA();
        void setConstantV(double _v);
        
        void scatter(const int & gap);

        Path()
        {
            precision = -1;//代表目前还是稀疏或不均匀的路径
            has_s = false;
            has_yaw = false;
            has_cur = false;
            has_v = false;
            has_t = false;
            has_width = false;
            has_a = false;
        }
    };

    typedef PathPoint* PathPointPtr;
    typedef Path* PathPtr;

    double distance(PathPoint _point1, PathPoint _point2);
    double orientation(PathPoint _point1, PathPoint _point2);
    bool isPointInPath(PathPoint _point, Path _path);
    void readFromCsv(planning::Path & _path, const char* _filename);
    void rowToString(std::vector<std::string> &record, const std::string& line, char delimiter);
} // namespace planning


#endif //TRAJECTORY_H





