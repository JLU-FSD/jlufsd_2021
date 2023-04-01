cmake_minimum_required(VERSION 3.5)

add_library(can_node_base can_node_base.cpp)
target_link_libraries(can_node_base CanInterface ${catkin_LIBRARIES})



add_library(can_node_qirui_eq can_node_qirui_eq.cpp)
target_link_libraries(can_node_qirui_eq CanInterface ${catkin_LIBRARIES})

add_library(can_tester_node can_tester_node.cpp)
target_link_libraries(can_tester_node CanInterface can_node_base ${catkin_LIBRARIES})



add_library(CanNode INTERFACE)
target_link_libraries(CanNode INTERFACE
                        can_node_base
                        can_node_qirui_eq 
                        can_tester_node
		)
