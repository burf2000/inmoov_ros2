"""Gazebo + RViz + sliders. Simulated robot, no hardware touched.

    ros2 launch inmoov_bringup sim.launch.py

Thin wrapper around inmoov_description/launch/gazebo.launch.py with the sliders switched
on, so the sim is reached the same way as everything else in this package.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo = os.path.join(
        get_package_share_directory("inmoov_description"), "launch", "gazebo.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo),
            launch_arguments={"use_gui": "true"}.items()),
    ])
