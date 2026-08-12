cmake_minimum_required(VERSION 3.8)
project(acadbot_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ---- localization_monitor: reports whether AMCL has converged ----
add_executable(localization_monitor src/localization_monitor.cpp)
ament_target_dependencies(localization_monitor rclcpp geometry_msgs tf2)

install(TARGETS
  localization_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
