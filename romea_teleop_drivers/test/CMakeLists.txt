find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_skid_steering_teleop_test skid_steering_teleop_test.cpp)
target_link_libraries(${PROJECT_NAME}_skid_steering_teleop_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_omni_steering_teleop_test omni_steering_teleop_test.cpp)
target_link_libraries(${PROJECT_NAME}_omni_steering_teleop_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_one_axle_steering_teleop_test one_axle_steering_teleop_test.cpp)
target_link_libraries(${PROJECT_NAME}_one_axle_steering_teleop_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_two_axle_steering_teleop_test_2_axes two_axle_steering_teleop_test_2_axes.cpp)
target_link_libraries(${PROJECT_NAME}_two_axle_steering_teleop_test_2_axes ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_two_axle_steering_teleop_test_4_axes two_axle_steering_teleop_test_4_axes.cpp)
target_link_libraries(${PROJECT_NAME}_two_axle_steering_teleop_test_4_axes ${PROJECT_NAME})
