cmake_minimum_required(VERSION 3.5)

add_executable(can2ros_and_csv_node can2ros_and_csv_node.cpp)
target_link_libraries(can2ros_and_csv_node CanInterface ${catkin_LIBRARIES})

add_executable(can_dbc_test can_dbc_test.cpp)
target_link_libraries(can_dbc_test CanInterface ${catkin_LIBRARIES})


#add_executable(enu_ref_point enu_ref_point.cpp)
#target_link_libraries(enu_ref_point CanNode ${catkin_LIBRARIES})

#add_executable(resample_realtime_rec resample_realtime_rec.cpp)
#target_link_libraries(resample_realtime_rec ${catkin_LIBRARIES})

#add_executable(real_time_record real_time_record.cpp)
#target_link_libraries(real_time_record ${catkin_LIBRARIES})
