cmake_minimum_required(VERSION 3.8)
project(acadbot_courier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(acadbot_courier_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(courier_core SHARED
  src/location_book.cpp
  src/job_registry.cpp
  src/nav_leg_client.cpp
)
target_include_directories(courier_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(courier_core
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(courier_node src/courier_node.cpp)
target_link_libraries(courier_node courier_core)
ament_target_dependencies(courier_node
  acadbot_courier_msgs
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action)

add_executable(delivery_client src/delivery_client.cpp)
ament_target_dependencies(delivery_client
  acadbot_courier_msgs
  rclcpp
  rclcpp_action)

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher
  geometry_msgs
  rclcpp
  tf2)

install(TARGETS
  courier_core
  courier_node
  delivery_client
  initial_pose_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(courier_core)
ament_export_dependencies(
  geometry_msgs nav2_msgs rclcpp rclcpp_action tf2 tf2_geometry_msgs tf2_ros)

ament_package()
