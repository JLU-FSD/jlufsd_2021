cmake_minimum_required(VERSION 2.8.3)

#平滑器
add_library(smoother_module src/common/smoother_module.cpp)#提供基础的框架与输入输出接口
target_link_libraries(smoother_module ${catkin_LIBRARIES})
target_link_libraries(smoother_module  iterative_polynomial_smoother)
target_link_libraries(smoother_module  polygonal_line)

add_library(smoother_common src/common/smoother_common.cpp)#提供基础的框架与输入输出接口
target_link_libraries(smoother_common planning_common ${catkin_LIBRARIES})

add_library(iterative_polynomial_smoother src/iterative_polynomial_smoother.cpp)#xy分离的迭代多项式平滑器
target_link_libraries(iterative_polynomial_smoother smoother_common ${catkin_LIBRARIES})

add_library(polygonal_line src/polygonal_line.cpp)#多边形平滑器
target_link_libraries(polygonal_line smoother_common ${catkin_LIBRARIES})