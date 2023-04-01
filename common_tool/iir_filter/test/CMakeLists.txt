cmake_minimum_required(VERSION 2.8)
enable_testing()
include_directories(
	
  ..
  ../iir
  )

set(test_iir_data_path "${CMAKE_CURRENT_SOURCE_DIR}/test_data/")  
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test_iir_data_path.h.in 
${CMAKE_CURRENT_SOURCE_DIR}/test_iir_data_path.h)

add_executable (test_butterworth butterworth.cpp)
target_link_libraries(test_butterworth iir_static)


add_executable (butterworth_test_data butterworth_test_data.cpp)
target_link_libraries(butterworth_test_data iir_static)
target_link_libraries(butterworth_test_data  ${PYTHON_LIBRARIES})
target_include_directories(butterworth_test_data PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(butterworth_test_data ${CMAKE_THREAD_LIBS_INIT})

add_test(TestButterworth test_butterworth)

add_executable (test_chebyshev1 chebyshev1.cpp)
target_link_libraries(test_chebyshev1 iir_static)
add_test(TestChebyshev1 test_chebyshev1)

add_executable (test_chebyshev2 chebyshev2.cpp)
target_link_libraries(test_chebyshev2 iir_static)
add_test(TestChebyshev2 test_chebyshev2)

add_executable (test_rbj rbj.cpp)
target_link_libraries(test_rbj iir_static)
add_test(TestRBJ test_rbj)


