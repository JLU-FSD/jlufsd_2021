cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

add_library(planning_core src/planning_core.cpp)
target_link_libraries(planning_core planning_common ${catkin_LIBRARIES})

#大学生无人方程式项目
add_subdirectory(FSA)