### Forked from https://github.com/MyRobotLab/inmoov_ros

Thank you Alan and MRL for their ROS1 / ROS 2 work.


### What Is It?
At the moment this is a ROS2 Rviz2/Gazebo display of an InMoov robot with joint control.
You can use Slider.py to move each joint in Gazebo


### Current system
- Ubuntu 24.04 LTS
- ROS2 Jammy 

### How to use it:

```bash

colcon build --symlink-install
source install/local_setup.bash
ros2 launch inmoov_description gazebo.launch.py use_gui:=false

```

This will fire up Gazebo, RVIZ2 and ROS2_Control


```bash
# stream a single joint at 50 Hz for 3 seconds (Ctrl+C to stop)
ros2 topic pub --rate 50 /inmoov_pos_cmd/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"


```
