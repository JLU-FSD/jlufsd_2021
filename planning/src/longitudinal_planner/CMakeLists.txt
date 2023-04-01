cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
#纵向规划器
add_library(longitudinal_module src/common/longitudinal_module.cpp)#提供基础的框架与输入输出接口
target_link_libraries(longitudinal_module forward_backwar_v_planner ${catkin_LIBRARIES})

add_library(longitudinal_common src/common/longitudinal_common.cpp)#通用组件
target_link_libraries(longitudinal_common planning_common ${catkin_LIBRARIES})

add_executable(longitudinal_test src/common/longitudinal_test.cpp)#测试程序
target_link_libraries(longitudinal_test longitudinal_module ${catkin_LIBRARIES})

#由Stanford DDL提出的基础最大速度规划器
add_library(forward_backwar_v_planner src/forward_backwar_v_planner.cpp)
target_link_libraries(forward_backwar_v_planner longitudinal_common ${catkin_LIBRARIES})







