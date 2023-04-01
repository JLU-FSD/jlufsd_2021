cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
)

add_library(fsa_planning src/fsa_planning.cpp)
target_link_libraries(fsa_planning planning_core longitudinal_module smoother_module ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(fsa_straight_planning src/fsa_straight_planning.cpp)
target_link_libraries(fsa_straight_planning fsa_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(straight_node src/main/straight_node.cpp)
# target_link_libraries(straight_node fsa_straight_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(fsa_skidpad_planning src/fsa_skidpad_planning.cpp)
target_link_libraries(fsa_skidpad_planning fsa_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(skidpad_node src/main/skidpad_node.cpp)
# target_link_libraries(skidpad_node fsa_skidpad_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(fsa_racetrack_planning src/fsa_racetrack_planning.cpp)
target_link_libraries(fsa_racetrack_planning fsa_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(racetrack_node src/main/racetrack_node.cpp)
# target_link_libraries(racetrack_node fsa_racetrsack_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(fsa_planning_node src/main/fsa_planning_node.cpp)
target_link_libraries(fsa_planning_node fsa_straight_planning fsa_skidpad_planning fsa_racetrack_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})





