cmake_minimum_required(VERSION 3.5)
project(deadlock)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(test_msgs REQUIRED)

add_executable(deadlock src/deadlock.cpp)
ament_target_dependencies(deadlock rclcpp rclcpp_action nav2_msgs)

add_executable(dummy_action_server src/dummy_action_server.cpp)
ament_target_dependencies(dummy_action_server rclcpp rclcpp_action nav2_msgs)

add_executable(dummy_action_server2 src/dummy_action_server2.cpp)
ament_target_dependencies(dummy_action_server2 rclcpp rclcpp_action nav2_msgs test_msgs)

add_executable(dummy_action_server3 src/dummy_action_server3.cpp)
ament_target_dependencies(dummy_action_server3 rclcpp rclcpp_action nav2_msgs test_msgs)

install(TARGETS
  deadlock dummy_action_server dummy_action_server2 dummy_action_server3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
