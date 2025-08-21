# CMakeLists.txt — ROS 2 Humble (ament)
cmake_minimum_required(VERSION 3.8)
project(inmoov_tools)

# If you add C++ targets and want a specific standard:
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
# Add explicit dependencies here as you need them, e.g.:
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

###########
## Build ##
###########
# Example C++ node (uncomment and edit when you add sources)
# add_executable(example_node src/example_node.cpp)
# ament_target_dependencies(example_node rclcpp)
# install(TARGETS example_node
#   DESTINATION lib/${PROJECT_NAME}
# )

#############
## Install ##
#############
# Install Python scripts (if you have any). Comment out if not using.
# Make sure the script files are executable (chmod +x).
# install(PROGRAMS
#   scripts/my_tool.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install data: launch files, configs, etc. (folders are optional)
install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# If you keep package resources (e.g., resource/inmoov_tools), install them:
# install(DIRECTORY resource/
#   DESTINATION share/${PROJECT_NAME}
#   OPTIONAL
# )

ament_package()
