cmake_minimum_required(VERSION 3.5)
find_library(ESD_DRIVERS_PATH ntcan)
find_package(Threads)
if(ESD_DRIVERS_PATH)
    message("esd_can_drivers is found in " ${ESD_DRIVERS_PATH})
    add_definitions(-D ESD_DRIVERS)
endif()

add_library(socket_can_channel socket_can_channel.cpp)
target_link_libraries(socket_can_channel can_dbc yaml-cpp ${CMAKE_THREAD_LIBS_INIT})

if(ESD_DRIVERS_PATH)
    add_library(esd_can_channel esd_can_channel.cpp)
    target_link_libraries(esd_can_channel can_dbc yaml-cpp ntcan)
    add_executable(test_esd_can_channel test/test_esd_can_channel.cpp)
    target_link_libraries(test_esd_can_channel esd_can_channel ${catkin_LIBRARIES})
    add_executable(test_esd test/test_esd.cpp)
    target_link_libraries(test_esd esd_can_channel ${catkin_LIBRARIES})
endif()

add_library(CanInterface INTERFACE)
target_link_libraries(CanInterface INTERFACE
						socket_can_channel)
if(ESD_DRIVERS_PATH)
    target_link_libraries(CanInterface INTERFACE
    esd_can_channel)
endif()

add_executable(test_socket_can test/test_socket_can.cpp)
target_link_libraries(test_socket_can socket_can_channel)



