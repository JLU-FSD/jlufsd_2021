#pragma once
#include <vector>
#include <math.h>

#include "trajectory.h"

struct CoordInfo
{
    double x;//新坐标原点在旧坐标系中的横轴位置
    double y;//新坐标原点在旧坐标系中的纵轴位置
    double theta;//逆时针旋转角度,弧度
};

class CoordTransformation 
{
public:
    CoordTransformation();

    //计算函数，采用先平移后旋转的方式
    planning::PathPoint         transform(const planning::PathPoint &_old_coord, const CoordInfo &_new_coord_info);
    planning::Path              transform(const planning::Path &_old_path, const CoordInfo &_new_coord_info);
    planning::PathPoint         inverseTransform(const planning::PathPoint &_old_point, const CoordInfo &_new_coord_info);
    planning::Path              inverseTransform(const planning::Path &_old_path, const CoordInfo &_new_coord_info);

private: 



};