cmake_minimum_required(VERSION 2.8.3)


#通用组件
add_library(planning_common src/planning_common.cpp)
target_link_libraries(planning_common planning_math_tool coord_transformation csv
                                      trajectory vehicle road trajectory_vis 
                                      ${catkin_LIBRARIES})

add_library(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})

add_library(trajectory_vis src/trajectory_vis.cpp)
target_link_libraries(trajectory_vis planning_math_tool ${catkin_LIBRARIES})

add_library(vehicle src/vehicle.cpp)
target_link_libraries(vehicle ${catkin_LIBRARIES})

add_library(road src/road.cpp)
target_link_libraries(road ${catkin_LIBRARIES})

add_library(coord_transformation src/coord_transformation.cpp)
target_link_libraries(coord_transformation ${catkin_LIBRARIES})

add_library(planning_math_tool src/planning_math_tool.cpp)
target_link_libraries(planning_math_tool ${catkin_LIBRARIES})

add_library(csv src/csv.cpp)
target_link_libraries(csv ${catkin_LIBRARIES})

