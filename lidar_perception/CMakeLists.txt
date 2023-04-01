cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  auto_msgs
)


#add_message_files(
#   FILES
#   sensor_gps.msg
#   output_point.msg
#   center_point.msg
#   Message2.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_perception
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/lidar_perception.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
   proj
 )


