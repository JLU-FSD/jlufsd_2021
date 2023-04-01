#include "coord_transformation.h"


CoordTransformation::CoordTransformation(){}


planning::PathPoint CoordTransformation::transform(const planning::PathPoint &_old_point, const CoordInfo &_new_coord_info)
{
    planning::PathPoint point;
    double delta_x = _old_point.x - _new_coord_info.x;
    double delta_y = _old_point.y - _new_coord_info.y;
    point.x = cos(_new_coord_info.theta) * delta_x + sin(_new_coord_info.theta) * delta_y;
    point.y = cos(_new_coord_info.theta) * delta_y - sin(_new_coord_info.theta) * delta_x;
    point.yaw = _old_point.yaw - _new_coord_info.theta;
    point.cur = _old_point.cur;
    return point;
}

planning::PathPoint CoordTransformation::inverseTransform(const planning::PathPoint &_old_point, const CoordInfo &_new_coord_info)
{
    planning::PathPoint point;

    double delta_x = cos(-_new_coord_info.theta) * _old_point.x + sin(-_new_coord_info.theta) * _old_point.y;
    double delta_y = cos(-_new_coord_info.theta) * _old_point.y - sin(-_new_coord_info.theta) * _old_point.x;

    point.x = delta_x +  _new_coord_info.x;
    point.y = delta_y + _new_coord_info.y;
    
    point.yaw = _old_point.yaw + _new_coord_info.theta;
    point.cur = _old_point.cur;
    return point;
}


planning::Path CoordTransformation::transform(const planning::Path &_old_path, const CoordInfo &_new_coord_info)
{
    planning::Path path;
    for(int i = 0; i < _old_path.path.size(); i++)
    {
        path.path.push_back( transform(_old_path.path[i], _new_coord_info) );
    }
    return path;
}

planning::Path CoordTransformation::inverseTransform(const planning::Path &_old_path, const CoordInfo &_new_coord_info)
{
    planning::Path path;
    for(int i = 0; i < _old_path.path.size(); i++)
    {
        path.path.push_back( inverseTransform(_old_path.path[i], _new_coord_info) );
    }
    return path;
}