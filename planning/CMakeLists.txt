cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  auto_msgs
  common_tool
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)
find_package(PCL REQUIRED)
find_package(PythonLibs 2.7) #need for matplotlib
find_package(yaml-cpp 0.6.3 REQUIRED)#needed  for mation_filter and mpc
#find_package (Threads)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# find_package(nanoflann REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  auto_msgs   geometry_msgs   nav_msgs   std_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planning
  CATKIN_DEPENDS auto_msgs common_tool geometry_msgs message_generation nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# set(rtk_data_path "${CMAKE_CURRENT_SOURCE_DIR}/data/")
# set(rtk_config_path "${CMAKE_CURRENT_SOURCE_DIR}/config/rtk_planning/")
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/rtk_data_path.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/rtk_data_path.h)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/longitudinal_planner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/longitudinal_planner/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/src/interface/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/planning_node/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/common) 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/longitudinal_planner) 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/smoother) 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/interface)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/planning_node)