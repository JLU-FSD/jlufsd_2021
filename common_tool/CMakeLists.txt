cmake_minimum_required(VERSION 2.8.3)
project(common_tool)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)
find_package(Threads)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include iir_filter math  time/include
  LIBRARIES common_tool 
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
add_library(common_tool INTERFACE)
target_link_libraries(common_tool INTERFACE
					 clampPidLib watchDogLib proto_file coordinate_trans
					  interpolation_1d interpolation_2d iir path					  
		)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/common_tool
  iir_filter
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/iir_filter)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/common_tool.cpp
# )
add_library(clampPidLib src/clampPid.cpp)
add_library(watchDogLib src/watchDog.cpp)
add_library(proto_file src/proto_file.cc)
target_link_libraries(proto_file ${Protobuf_LIBRARIES})
add_library(coordinate_trans src/coordinate_trans.cpp)
add_library(interpolation_1d src/interpolation_1d.cpp)
add_library(interpolation_2d src/interpolation_2d.cpp)
add_library(path src/path.cpp)

add_executable(factory_test test/factory_test.cpp)
add_executable(test_interpolation test/test_interpolation.cpp)


add_executable(test_mlinterp test/test_mlinterp.cpp)
add_executable(test_csv test/test_csv.cpp)

set(test_data_path "${CMAKE_CURRENT_SOURCE_DIR}/test/")  
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test/test_data_path.h.in 
${CMAKE_CURRENT_SOURCE_DIR}/test/test_data_path.h)

target_link_libraries(test_interpolation interpolation_1d)
target_link_libraries(test_interpolation interpolation_2d)

string(REGEX REPLACE "(.+)\\src.*" "\\1" AUTO_ROS_BASE_PATH ${PROJECT_SOURCE_DIR})
set(auto_ros_base_path ${AUTO_ROS_BASE_PATH})  
string(REGEX REPLACE "(.+)\\common_tool.*" "\\1" AUTO_ROS_SRC_PATH ${PROJECT_SOURCE_DIR})
set(auto_ros_src_path ${AUTO_ROS_SRC_PATH})  
set(auto_ros_control_path ${AUTO_ROS_SRC_PATH}control/)  
set(auto_ros_localization_path ${AUTO_ROS_SRC_PATH}localization/)  

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/auto_ros_base_path.h.in 
${CMAKE_CURRENT_SOURCE_DIR}/include/auto_ros_base_path.h)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/common_tool_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_geometry test/test_geometry.cpp)
 if(TARGET test_geometry)
   target_link_libraries(test_geometry clampPidLib)
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
