#ifndef ROAD_H
#define ROAD_H

#include "trajectory.h"

namespace planning
{
    struct SLPoint
    {
        double                          s;
        double                          l;
        double                          yaw;
        double                          cur;
        double                          distance;
        
        double                          left_width;
        double                          right_width;
        
        double                          vx;//纵向速度
        double                          ax;//纵向加速度

        SLPoint()
        {
            s = 0;
            l = 0;
            yaw = 0;
            cur = 0;
            left_width = 0;
            right_width = 0;
            vx = 0;//纵向速度
            distance = 0;//路程
            ax = 0;//纵向加速度
        }
    };

    struct SLPath
    {
        std::vector<SLPoint>          path;
        void showInCsv(const char* _filename);
    };
    
    struct Road
    {
        planning::Path                  left_boundry;
        planning::Path                  right_boundry;
        planning::Path                  center_line;

        Road()
        {
            left_boundry.path.clear();
            right_boundry.path.clear();
            center_line.path.clear();
        }
        void                    calculateRoadWidth();//计算道路宽度
        void                    calculateRoadWidth(planning::Path & path);//计算道路宽度
        void                    generateCenterLine();//生成道路中心线

        planning::SLPoint       toSL(const planning::PathPoint & _p);//可以用kdtree进行查找
        planning::PathPoint     toXY(const planning::SLPoint & _p);
        planning::SLPath        toSL(const planning::Path & _p);//可以用kdtree进行查找
        planning::Path          toXY(const planning::SLPath & _p);
    };

    typedef Road* RoadPtr;
    
}

#endif //ROAD_H
