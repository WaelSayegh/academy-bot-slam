cmake_minimum_required(VERSION 3.8)
project(acadbot_courier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(acadbot_courier_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# ---- courier_server: the state-machine engine ----
add_executable(courier_server
  src/courier_server.cpp
  src/location_table.cpp
  src/location_markers.cpp)
target_include_directories(courier_server PRIVATE include)
ament_target_dependencies(courier_server
  rclcpp rclcpp_action acadbot_courier_msgs nav2_msgs
  geometry_msgs tf2 tf2_geometry_msgs visualization_msgs)

# ---- courier_bt_server: the behaviour-tree engine (bonus) ----
add_executable(courier_bt_server
  src/courier_bt_server.cpp
  src/bt_nodes.cpp
  src/location_table.cpp
  src/location_markers.cpp)
target_include_directories(courier_bt_server PRIVATE include)
ament_target_dependencies(courier_bt_server
  rclcpp rclcpp_action acadbot_courier_msgs nav2_msgs
  geometry_msgs tf2 tf2_geometry_msgs visualization_msgs
  ament_index_cpp behaviortree_cpp)

# ---- initial_pose_seeder: tells AMCL where the robot starts ----
add_executable(initial_pose_seeder src/initial_pose_seeder.cpp)
ament_target_dependencies(initial_pose_seeder
  rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS courier_server courier_bt_server initial_pose_seeder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
