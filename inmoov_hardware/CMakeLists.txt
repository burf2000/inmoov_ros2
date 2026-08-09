cmake_minimum_required(VERSION 3.16)
project(inmoov_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CURL REQUIRED)

# yaml-cpp: prefer the vendor package, fall back to the system one. Jazzy on Ubuntu 24.04
# ships yaml-cpp 0.8 which exports the yaml-cpp::yaml-cpp target.
find_package(yaml-cpp REQUIRED)

# --- shared mapping + MRL transport --------------------------------------------------
# Deliberately a separate library so the SystemInterface plugin and the /joint_states
# relay node link the SAME mapping arithmetic. There is no second copy of it anywhere.
add_library(inmoov_mapping SHARED
  hardware/joint_map.cpp
  hardware/mrl_client.cpp
)
target_compile_features(inmoov_mapping PUBLIC cxx_std_17)
target_include_directories(inmoov_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(inmoov_mapping PUBLIC yaml-cpp::yaml-cpp CURL::libcurl)

# --- the ros2_control plugin ---------------------------------------------------------
add_library(inmoov_hardware SHARED
  hardware/inmoov_system.cpp
)
target_compile_features(inmoov_hardware PUBLIC cxx_std_17)
target_include_directories(inmoov_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(inmoov_hardware PUBLIC inmoov_mapping)
ament_target_dependencies(inmoov_hardware PUBLIC
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface inmoov_hardware.xml)

# --- /joint_states relay (the RViz slider path) --------------------------------------
add_executable(joint_state_relay src/joint_state_relay_node.cpp)
target_link_libraries(joint_state_relay PRIVATE inmoov_mapping)
ament_target_dependencies(joint_state_relay PUBLIC
  ament_index_cpp
  rclcpp
  sensor_msgs
)

# --- install --------------------------------------------------------------------------
install(TARGETS inmoov_mapping inmoov_hardware
  EXPORT export_inmoov_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS joint_state_relay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Pure arithmetic tests over the SHIPPED joint map. These must pass before anything is
  # allowed near the physical robot.
  ament_add_gtest(test_joint_map test/test_joint_map.cpp)
  target_link_libraries(test_joint_map inmoov_mapping)
  target_compile_definitions(test_joint_map PRIVATE
    INMOOV_JOINT_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/inmoov_joint_map.yaml")
endif()

ament_export_include_directories(include)
ament_export_libraries(inmoov_mapping inmoov_hardware)
ament_export_targets(export_inmoov_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
