cmake_minimum_required(VERSION 3.0.2)
#离线规划器
add_library(interface_with_automsgs src/interface_with_automsgs.cpp)#平滑器测试程序
target_link_libraries(interface_with_automsgs planning_common ${catkin_LIBRARIES})