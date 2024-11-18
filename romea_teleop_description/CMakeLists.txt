cmake_minimum_required(VERSION 3.5)
project(romea_teleop_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_python_install_module(python/romea_teleop_description.py)

find_package(ament_lint_auto REQUIRED)
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
ament_lint_auto_find_test_dependencies()

find_package(ament_cmake_pytest REQUIRED)
configure_file(test/test_skid_steering_base_info.yaml  test/test_skid_steering_base_info.yaml)
configure_file(test/test_omni_steering_base_info.yaml  test/test_omni_steering_base_info.yaml)
configure_file(test/test_one_axle_steering_base_info.yaml  test/test_one_axle_steering_base_info.yaml)
configure_file(test/test_two_axle_steering_base_info.yaml  test/test_two_axle_steering_base_info.yaml)
configure_file(test/test_skid_steering_teleop_config.yaml  test/test_skid_steering_teleop_config.yaml)
configure_file(test/test_omni_steering_teleop_config.yaml  test/test_omni_steering_teleop_config.yaml)
configure_file(test/test_one_axle_steering_teleop_config.yaml  test/test_one_axle_steering_teleop_config.yaml)
configure_file(test/test_two_axle_steering_teleop_config.yaml  test/test_two_axle_steering_teleop_config.yaml)

ament_add_pytest_test(test_teleop_description test/test_teleop_description.py)

ament_package()
