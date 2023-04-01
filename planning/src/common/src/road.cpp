#include "road.h"

void planning::Road::calculateRoadWidth()
{
    //当有了稠密地图之后就比较好算了,直接用kdtree找最短距离即可,chrono测试时间消耗为0.01ms数量级
    for(auto &path_point: center_line.path)
    {
        const auto it_nearest_left = std::min_element(left_boundry.path.begin(), left_boundry.path.end(),
                                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(path_point.x - a.x, path_point.y - a.y);
                                                    const double db = std::hypot(path_point.x - b.x, path_point.y - b.y);

                                                    return da < db;
                                                });
        path_point.left_width = std::hypot(it_nearest_left->x - path_point.x, 
                                            it_nearest_left->y - path_point.y);

        const auto it_nearest_right = std::min_element(right_boundry.path.begin(), right_boundry.path.end(),
                                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(path_point.x - a.x, path_point.y - a.y);
                                                    const double db = std::hypot(path_point.x - b.x, path_point.y - b.y);

                                                    return da < db;
                                                });
        path_point.right_width = std::hypot(it_nearest_right->x - path_point.x, 
                                            it_nearest_right->y - path_point.y);
    }
}

void planning::Road::calculateRoadWidth(planning::Path & path)
{
    //当有了稠密地图之后就比较好算了,直接用kdtree找最短距离即可,chrono测试时间消耗为0.01ms数量级
    for(auto &path_point: path.path)
    {
        const auto it_nearest_left = std::min_element(left_boundry.path.begin(), left_boundry.path.end(),
                                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(path_point.x - a.x, path_point.y - a.y);
                                                    const double db = std::hypot(path_point.x - b.x, path_point.y - b.y);

                                                    return da < db;
                                                });
        path_point.left_width = std::hypot(it_nearest_left->x - path_point.x, 
                                            it_nearest_left->y - path_point.y);

        const auto it_nearest_right = std::min_element(right_boundry.path.begin(), right_boundry.path.end(),
                                                    [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(path_point.x - a.x, path_point.y - a.y);
                                                    const double db = std::hypot(path_point.x - b.x, path_point.y - b.y);

                                                    return da < db;
                                                });
        path_point.right_width = std::hypot(it_nearest_right->x - path_point.x, 
                                            it_nearest_right->y - path_point.y);
    }
    path.has_width = true;
}

void planning::Road::generateCenterLine()
{
    if (left_boundry.path.size() == 0 || right_boundry.path.size() == 0)
    {
        std::cout<<"The num of boundry points is 0!"<<std::endl;
        return;
    }
    
    center_line.path.clear();
    for (const auto &right_point: right_boundry.path) {
        //找出距离当前左侧车道点最近的点
        const auto it_left = std::min_element(left_boundry.path.begin(), left_boundry.path.end(),
                                                [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                                {
                                                    const double da = std::hypot(right_point.x - a.x, right_point.y - a.y);
                                                    const double db = std::hypot(right_point.x - b.x, right_point.y - b.y);

                                                    return da < db;
                                                });
        // if(std::hypot(right_point.x - it_left->x, right_point.y - it_left->y) < 3 *1.8
        // &&std::hypot(right_point.x - it_left->x, right_point.y - it_left->y) > 3*0.5)
        // {
        //     planning::PathPoint p;
        //     p.x = static_cast<float>((right_point.x + it_left->x) / 2.0);
        //     p.y = static_cast<float>((right_point.y + it_left->y) / 2.0);
        //     center_line.path.push_back(p);
        // }
        planning::PathPoint p;
        p.x = static_cast<float>((right_point.x + it_left->x) / 2.0);
        p.y = static_cast<float>((right_point.y + it_left->y) / 2.0);
        center_line.path.push_back(p);
    }

}

planning::SLPoint planning::Road::toSL(const planning::PathPoint & _p)
{//可以用kdtree进行查找
    const auto it_nearest = std::min_element(center_line.path.begin(), center_line.path.end(),
                                                [&](const planning::PathPoint &a, const planning::PathPoint &b) 
                                            {
                                                const double da = std::hypot(_p.x - a.x, _p.y - a.y);
                                                const double db = std::hypot(_p.x - b.x, _p.y - b.y);
                                                return da < db;
                                            });
    planning::SLPoint p;
    p.s = it_nearest->s;
    p.l = std::hypot(it_nearest->x - _p.x, it_nearest->y - _p.y);
    p.cur += _p.cur;
    return p;
}

planning::PathPoint planning::Road::toXY(const planning::SLPoint & _p)
{
    int i = 0;
    for(; i < center_line.path.size() - 1; i++)
    {
        if(_p.s < center_line.path[i + 1].s)break;
    }
    double s = _p.s - center_line.path[i].s;
    double k = s / (center_line.path[i + 1].s - center_line.path[i].s);//比例，越小越接近前点
    planning::PathPoint p;
    p.x = (1 - k) * center_line.path[i].x + k * center_line.path[i + 1].x;
    p.y = (1 - k) * center_line.path[i].y + k * center_line.path[i + 1].y;
    p.yaw = (1 - k) * center_line.path[i].yaw + k * center_line.path[i + 1].yaw;
    p.x += _p.l * cos(p.yaw + M_PI_2);
    p.y += _p.l * sin(p.yaw + M_PI_2);
    p.cur += _p.cur;
    p.left_width = _p.left_width;
    p.right_width = _p.right_width;
    return p;
}

planning::SLPath planning::Road::toSL(const planning::Path & _path)
{//可以用kdtree进行查找
    planning::SLPath path;
    for(auto &p: _path.path)
    {
        path.path.push_back(toSL(p));
    }
    return path;
}

planning::Path planning::Road::toXY(const planning::SLPath & _path)
{
    planning::Path path;
    for(auto &p: _path.path)
    {
        path.path.push_back(toXY(p));
    }
    return path;
}

void planning::SLPath::showInCsv(const char* _filename)
{
    std::ofstream outFile;  
    outFile.open(_filename, std::ios::out); // 打开模式决定是继续写还是覆盖：out是覆盖，app是续写  
    outFile << "s" << ','
            << "l" << ','
            << "yaw" << ','
            << "cur" << ','
            << "distance" << ','
            << "left_width" << ','
            << "right_width" << ','
            << "vx" << ','
            << "ax" << ','<<std::endl;
            
    for(int i = 0; i < path.size(); i++)
    {
        outFile << path[i].s << ','
                << path[i].l << ','
                << path[i].yaw << ','
                << path[i].cur << ','
                << path[i].distance << ','
                << path[i].left_width << ','
                << path[i].right_width << ','
                << path[i].vx << ','
                << path[i].ax << ','
                <<std::endl;
    }
    outFile.close();
}