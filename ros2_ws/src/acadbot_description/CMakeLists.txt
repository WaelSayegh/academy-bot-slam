cmake_minimum_required(VERSION 3.8)
project(acadbot_description)

find_package(ament_cmake REQUIRED)

# Install the description assets so other packages can $(find ...) them.
install(
  DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
