cmake_minimum_required(VERSION 3.8)
project(acadbot_courier_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDelivery.srv"
  "action/ExecuteDelivery.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
