"""RViz + sliders. Nothing is connected — the model moves, no robot does.

    ros2 launch inmoov_bringup view.launch.py

Use this to check the URDF, meshes and joint limits without a robot anywhere near it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("inmoov_description")
    model = PathJoinSubstitution([pkg, "robots", "inmoov.urdf.xacro"])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(
                 Command(["xacro ", model, " use_real_hardware:=false"]), value_type=str)}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution([pkg, "config", "inmoov.rviz"])]),
    ])
