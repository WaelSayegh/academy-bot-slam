cmake_minimum_required(VERSION 3.8)
project(acadbot_courier_pipeline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(acadbot_courier_interfaces REQUIRED)

add_executable(courier_manager src/courier_manager.cpp)
ament_target_dependencies(courier_manager
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  acadbot_courier_interfaces
)

install(TARGETS courier_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/courier_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME courier_cli
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
