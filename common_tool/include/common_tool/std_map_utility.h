#pragma once
#include <map>
#include <iostream>
namespace auto_ros
{
namespace common_tool
{
using double_map = std::map<std::string, double>;
using str_map = std::map<std::string, std::string>;
//using map_iter = std::map<std::string, double>::iterator;
template <class T>
T find_in_map(std::string which_var, const std::map<std::string, T> &in_map)
{
    auto temp_iter = in_map.find(which_var);
    if (temp_iter != in_map.end())
    {
        return temp_iter->second;
    }
    else
    {
        std::cout << "\033[31m Error:find_in_map<double> failed and key name is:" << which_var << std::endl;
        throw "find_in_map<double> failed";
    }
}
} // namespace common_tool
} // namespace auto_ros