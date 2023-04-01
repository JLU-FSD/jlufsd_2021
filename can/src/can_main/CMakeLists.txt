cmake_minimum_required(VERSION 3.5)

add_executable(main_can main_can.cpp)
target_link_libraries(main_can CanNode ${catkin_LIBRARIES})
