cmake_minimum_required(VERSION 3.5)
project(romea_teleop_drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_cmd_mux_msgs REQUIRED)
find_package(romea_joystick_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_cmd_mux_msgs REQUIRED)
find_package(romea_cmd_mux_utils REQUIRED)


add_library(${PROJECT_NAME} SHARED
 src/skid_steering_teleop.cpp
 src/omni_steering_teleop.cpp
 src/one_axle_steering_teleop.cpp
 src/two_axle_steering_teleop.cpp
 src/teleop_base.cpp
 src/command_parameters.cpp
 src/joystick_parameters.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
 PRIVATE "ROMEA_TELEOP_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 rclcpp_components
 romea_core_common
 romea_core_mobile_base
 romea_common_utils
 romea_cmd_mux_msgs
 romea_joystick_utils
 romea_mobile_base_msgs
 romea_mobile_base_utils
 romea_cmd_mux_msgs
 romea_cmd_mux_utils
 geometry_msgs
 )

rclcpp_components_register_nodes(${PROJECT_NAME}
  "romea::SkidSteeringTeleop"
  "romea::OmniSteeringTeleop"
  "romea::OneAxleSteeringTeleop"
  "romea::TwoAxleSteeringTeleop"
  )

install(TARGETS
 ${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)

add_executable(skid_steering_teleop_node src/skid_steering_teleop_node.cpp)
target_link_libraries(skid_steering_teleop_node ${PROJECT_NAME})
ament_target_dependencies(skid_steering_teleop_node rclcpp)

add_executable(omni_steering_teleop_node src/omni_steering_teleop_node.cpp)
target_link_libraries(omni_steering_teleop_node ${PROJECT_NAME})
ament_target_dependencies(omni_steering_teleop_node rclcpp)

add_executable(one_axle_steering_teleop_node src/one_axle_steering_teleop_node.cpp)
target_link_libraries(one_axle_steering_teleop_node ${PROJECT_NAME})
ament_target_dependencies(one_axle_steering_teleop_node rclcpp)

add_executable(two_axle_steering_teleop_node src/two_axle_steering_teleop_node.cpp)
target_link_libraries(two_axle_steering_teleop_node ${PROJECT_NAME})
ament_target_dependencies(two_axle_steering_teleop_node rclcpp)

install(TARGETS
 skid_steering_teleop_node
 omni_steering_teleop_node
 one_axle_steering_teleop_node
 two_axle_steering_teleop_node
 DESTINATION lib/${PROJECT_NAME})

#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
#ament_export_dependencies(geometry_msgs)
#ament_export_dependencies(romea_corecommon)
#ament_export_dependencies(romea_common_utils)
#ament_export_dependencies(four_wheel_steering_msgs)
#ament_export_dependencies(ackermann_msgs)
#ament_export_dependencies(romea_mobile_base)
#ament_export_dependencies(romea_mobile_base_msgs)
#ament_export_dependencies(romea_mobile_base_utils)
#ament_export_dependencies(romea_localisation_msgs)

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

#install(TARGETS
# ${PROJECT_NAME}
# DESTINATION lib/${PROJECT_NAME})

install(
 DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
 )

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

#install(
# TARGETS ${PROJECT_NAME}
# EXPORT export_${PROJECT_NAME}
# LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION bin
# INCLUDES DESTINATION include
# )

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()
