cmake_minimum_required(VERSION 3.8)
project(digirunner_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(digirunner_interfaces REQUIRED)

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

# servo_driver_node: all six bus servos through the BusLinker (#7)
add_executable(servo_driver_node src/servo_driver_node.cpp)
target_include_directories(servo_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(servo_driver_node PUBLIC cxx_std_17)
target_link_libraries(servo_driver_node
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS})
install(TARGETS servo_driver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
