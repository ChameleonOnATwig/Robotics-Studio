cmake_minimum_required(VERSION 3.8)
project(digirunner_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(digirunner_interfaces REQUIRED)

# classical_controller: imu/data + joint_states -> joint_commands, serves /walk
add_executable(classical_controller src/classical_controller.cpp)
target_include_directories(classical_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(classical_controller PUBLIC cxx_std_17)
target_link_libraries(classical_controller
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${digirunner_interfaces_TARGETS})

install(TARGETS classical_controller DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
