cmake_minimum_required(VERSION 3.16)
project(inmoov_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# The slider -> controller bridge. Installed into lib/ so `ros2 launch` can find it as an
# executable of this package.
install(PROGRAMS scripts/slider_to_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
