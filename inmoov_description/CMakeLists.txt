cmake_minimum_required(VERSION 3.5)
project(inmoov_description)

find_package(ament_cmake REQUIRED)

# Install all non-compiled assets so package:// works in RViz/Gazebo
install(DIRECTORY
  urdf
  meshes
  robots
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()