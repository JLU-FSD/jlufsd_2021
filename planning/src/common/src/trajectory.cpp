#include "trajectory.h"

double planning::distance(PathPoint _point1, PathPoint _point2)
{
    return sqrt(pow(_point1.x - _point2.x, 2) + pow(_point1.y - _point2.y, 2));
}

double planning::orientation(PathPoint _point1, PathPoint _point2)
{
    return atan2(_point2.y - _point1.y, _point2.x - _point1.x);
}

void planning::Path::showInCsv(const char* _filename)
{
    std::ofstream outFile;  
    outFile.open(_filename, std::ios::out); // 打开模式决定是继续写还是覆盖：out是覆盖，app是续写  
    outFile << "x" << ','
            << "y" << ','
            << "yaw" << ','
            << "cur" << ','
            << "s" << ','
            << "left_width" << ','
            << "right_width" << ','
            << "vx" << ','
            << "ax" << ','
            << "t" << ','
            <<std::endl;
            
    for(int i = 0; i < path.size(); i++)
    {
        outFile << path[i].x << ','
                << path[i].y << ','
                << path[i].yaw << ','
                << path[i].cur << ','
                << path[i].s << ','
                << path[i].left_width << ','
                << path[i].right_width << ','
                << path[i].vx << ','
                << path[i].ax << ','
                << path[i].t << ','
                <<std::endl;
    }
    outFile.close();
}

void planning::Path::calculateS()
{
    if(path.size() <= 0)
    {
        return;
    }
    double s = 0;
    for(int i = 0; i < path.size(); i++)
    {
        if(i == 0)
        {
            path[i].s = s; 
            continue;
        }
        s += planning::distance(path[i],path[i - 1]);
        path[i].s = s;        
    }
    has_s = true;
}

void planning::Path::calculateYaw()
{
    if(path.size() <= 0)return;
    //对航向角进行提取
    for(int i = 0; i < path.size()-1; i++)
    {
        path[i].yaw = atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);// * M_1_PI * 180;
        // ROS_INFO("yaw = %f", path[i].yaw);
    }
    //最后一个怎么办?令最后一个与倒数第二个相等
    path[path.size() - 1].yaw = path[path.size() - 2].yaw;
    has_yaw = true;
}

void planning::Path::calculateCur()
{//生成曲率的前提是具备航向角,和路程
    if(path.size() <= 0)return;
    if(!has_yaw || !has_s)
    {
        std::cout<< "Need yaw and s to calculate cur!"<<std::endl;
        return;
    }
    for(int i = 0; i < path.size()-2; i++)
    {
        path[i].cur = planning::angleCorrection(path[i+1].yaw - path[i].yaw) / (path[i+1].s - path[i].s);
        // ROS_INFO("yaw = %f", path[i].yaw);
    }
    //最后一个怎么办?令最后一个与倒数第二个相等
    path[path.size() - 2].yaw = path[path.size() - 3].yaw;
    path[path.size() - 1].yaw = path[path.size() - 2].yaw;
    has_cur = true;
}

void planning::Path::calculateCur2(const int & offset)
{//适合提取yaw噪声较大的路径曲率
    //采用外接圆直径的方式计算曲率
    double cur, a, b, c;
    for(int i = offset; i < path.size() - offset; i++)
    {
        a = distance(path[i - offset], path[i]);
        b = distance(path[i], path[i + offset]);
        c = distance(path[i - offset], path[i + offset]);
        cur = planning::triangleCircumCurvature(a, b, c);

        double theta_AB = atan2(path[i].y - path[i - offset].y, path[i].x - path[i - offset].x);
        double theta_AC = atan2(path[i + offset].y - path[i - offset].y, path[i + offset].x - path[i - offset].x);

        if(cur < 0.0002)
        {
            path[i].cur = 0;
        }
        else
        {
            if((theta_AC - theta_AB) < 0)
            {
                path[i].cur = cur * (-1);
            }
            else
            {
                path[i].cur = cur;
            }
        }
    }
}

void planning::Path::calculateT()
{
    if(!has_s || !has_v)
    {
        std::cout<< "Need s and v to calculate t!"<<std::endl;
        return;
    }
    for(int i = 1; i < path.size(); i++)
    {
        if(path[i - 1].vx <= 0)
        {
            std::cout<< "Can not calculate t when v is 0."<<std::endl;
            return;
        }
        path[i].t = path[i - 1].t + (path[i].s - path[i - 1].s) / path[i - 1].vx;
    }
    has_t = true;
}

void planning::Path::calculateA()
{
    if(!has_t || !has_v)
    {
        std::cout<< "Need t and v to calculate t!"<<std::endl;
        return;
    }
    for(int i = 0; i < path.size() - 1; i++)
    {
        if(path[i - 1].vx <= 0)
        {
            std::cout<< "Can not calculate t when v is 0."<<std::endl;
            return;
        }
        path[i].ax = (path[i + 1].vx - path[i].vx) / (path[i + 1].t - path[i].t);
    }
    has_a = true;
}

void planning::Path::setConstantV(double _v)
{
    for(auto &path_point: path)
    {
        path_point.vx = _v;
    }
}

void planning::Path::scatter(const int & gap)
{
    std::vector<PathPoint> scattered_path;
    for(int i = 0; i < path.size(); i++)
    {
        if(i%gap == 0)
        {
            scattered_path.push_back(path[i]);
        }
    }
    path = scattered_path;
}

void planning::readFromCsv(planning::Path & _path, const char* _filename)
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

        planning::PathPoint point;
        point.x = atof( row_element[0].c_str() );
        point.y = atof( row_element[1].c_str() );
        point.yaw = atof( row_element[2].c_str() );
        point.cur = atof( row_element[3].c_str() );
        point.s = atof( row_element[4].c_str() );
        point.left_width = atof( row_element[5].c_str() );
        point.right_width = atof( row_element[6].c_str() );
        point.vx = atof( row_element[5].c_str() );
        point.ax = atof( row_element[6].c_str() );
        point.t = atof( row_element[7].c_str() );
        _path.path.push_back(point);
    }

    in.close();
}

void planning::rowToString(std::vector<std::string> &record, const std::string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax=line.length();
    std::string curstring;
    record.clear();
    while(linepos<linemax)
    {
        c = line[linepos];
        if(isdigit(c)||c=='.'||c=='-'){
            curstring+=c;
        }
        else if(c==delimiter&&curstring.size()){
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
}//end row_to_string_vector function

bool planning::isPointInPath(planning::PathPoint _point, planning::Path _path)
{
    for(const auto &path_point: _path.path)
    {
        if(planning::distance(path_point, _point) < 0.5 )
        {
            return true;
        }
    }
    return false;    
}