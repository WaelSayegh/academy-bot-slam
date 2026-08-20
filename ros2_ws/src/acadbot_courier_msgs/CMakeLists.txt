cmake_minimum_required(VERSION 3.8)
project(acadbot_courier_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Field contents are TODO (see architecture/architecture.md §7) — this just
# wires the three files into the build. It won't generate anything useful
# until fields are added, but it will compile as-is (empty definitions).
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDelivery.srv"
  "action/ExecuteDelivery.action"
  "msg/NamedLocation.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
