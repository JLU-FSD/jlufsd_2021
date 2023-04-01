#include "interface_with_automsgs.h"

namespace planning {

auto_msgs::planning toAutoMsgs(Path _path)
{
    auto_msgs::planning p;
    for(int i = 0; i < _path.path.size(); i++)
    {
        p.path.push_back(toAutoMsgs(_path.path[i]));
    }
    return p;

}


auto_msgs::planning_point toAutoMsgs(PathPoint _p)
{
    auto_msgs::planning_point p;
    p.x = _p.x;
    p.y = _p.y;
    p.v = _p.vx;
    p.a = _p.ax;
    p.kappa = _p.cur;
    p.theta = _p.yaw * 180 * M_1_PI;
    p.s = _p.s;
    return p;

}


Path fromAutoMsgs(auto_msgs::planning _path)
{
    Path p;
    for(int i = 0; i < _path.path.size(); i++)
    {
        p.path.push_back(fromAutoMsgs(_path.path[i]));
        if(p.has_s == false)    if(p.path[i].s > 0)     p.has_s = true;
        if(p.has_v == false)    if(p.path[i].vx > 0)    p.has_v = true;
        if(p.has_a == false)    if(p.path[i].ax != 0)   p.has_a = true;
        if(p.has_yaw == false)  if(p.path[i].yaw != 0)  p.has_yaw = true;
        if(p.has_cur == false)  if(p.path[i].cur != 0)  p.has_cur = true;
    }
    return p;
}


PathPoint fromAutoMsgs(auto_msgs::planning_point _p)
{
    PathPoint p;
    p.x = _p.x;
    p.y = _p.y;
    p.vx = _p.v;
    p.ax = _p.a;
    p.cur = _p.kappa;
    p.yaw = _p.theta * M_PI / 180;
    p.s = _p.s;
    return p;
}

double distance(auto_msgs::planning_point _point1,auto_msgs::planning_point _point2)
{
    return sqrt(pow(_point1.x - _point2.x, 2) + pow(_point1.y - _point2.y, 2));
}

void readFromCsv(auto_msgs::planning & _path, const char* _filename)
{
    _path.path.clear();

    std::vector<std::string> row_element;
    std::string line;
    std::ifstream in(_filename,std::ios::in);

    if (in.fail())  { std::cout << "File not found" <<std::endl;return;}
    bool is_first_row = true;
    while(getline(in, line)  && in.good() )
    {   
        if(is_first_row == true)
        {
            is_first_row = false;
            continue;
        }
        
        //把line里的单元格数字字符提取出来，“,”为单元格分隔符                              
        planning::rowToString(row_element, line, ','); 

        auto_msgs::planning_point point;
        point.x = atof( row_element[0].c_str() );
        point.y = atof( row_element[1].c_str() );
        point.theta = atof( row_element[2].c_str() );
        point.kappa = atof( row_element[3].c_str() );
        point.v = atof( row_element[4].c_str() );
        point.s = atof( row_element[5].c_str() );
        point.a = atof( row_element[6].c_str() );
        _path.path.push_back(point);
    }

    in.close();
}
void showInCsv(auto_msgs::planning & _path ,const char* _name)
{
    // 写文件  
    std::ofstream outFile;  
    outFile.open(_name, std::ios::out); // 打开模式可省略  
    outFile << "x" << ','
            << "y" << ','
            << "theta" << ','
            << "kappa" << ','
            << "v" << ','
            << "s" << ','
            << "a" << ','
            <<std::endl;

    for(int i = 0; i < _path.path.size(); i++)
    {
        outFile << _path.path[i].x << ','
                << _path.path[i].y << ','
                << _path.path[i].theta << ','
                << _path.path[i].kappa << ','
                << _path.path[i].v << ','
                << _path.path[i].s << ','
                << _path.path[i].a << ','
                <<std::endl;
    }
    outFile.close(); 
}



}