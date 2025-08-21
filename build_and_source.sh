#!/bin/bash
# build_and_source.sh

colcon build --symlink-instal
source install/setup.bash
ros2 launch inmoov_description display.launch.py
