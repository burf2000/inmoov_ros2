"""The same real robot, driven through ros2_control instead of the direct relay.

    ros2 launch inmoov_bringup controller.launch.py                  # for real
    ros2 launch inmoov_bringup controller.launch.py dry_run:=true    # log only

You want real.launch.py for day-to-day slider driving. Use this one when something else
needs to command the robot the standard ROS way — a trajectory, MoveIt, another node —
because here the joints are exposed as a proper controller:

    ros2 topic pub /inmoov_position_controller/commands std_msgs/Float64MultiArray ...

The sliders still work. In this mode /joint_states belongs to joint_state_broadcaster (the
robot reporting where it IS), so the sliders are remapped to slider_joint_states and the
slider_to_controller bridge forwards them to the controller's command topic.

SAFETY: exactly one thing may drive MyRobotLab at a time. Stop inmoov-driver.service on
pi5-inmoov first, start it again afterwards.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _controller_joints():
    """The controller's joint list, IN ORDER.

    The command topic is a bare array of numbers with no names attached, so the bridge must
    publish in exactly this order. Reading the same YAML the controller reads is the only
    way to stop the two drifting apart silently.
    """
    path = os.path.join(
        get_package_share_directory("inmoov_bringup"), "config", "inmoov_controllers.yaml")
    with open(path) as fh:
        cfg = yaml.safe_load(fh)
    return list(cfg["inmoov_position_controller"]["ros__parameters"]["joints"])


def generate_launch_description():
    description_pkg = FindPackageShare("inmoov_description")
    bringup_pkg = FindPackageShare("inmoov_bringup")
    model = PathJoinSubstitution([description_pkg, "robots", "inmoov.urdf.xacro"])

    dry_run = LaunchConfiguration("dry_run")
    enabled_groups = LaunchConfiguration("enabled_groups")

    # ParameterValue(..., value_type=str) is mandatory: without it launch tries to parse the
    # whole URDF as YAML and dies before anything starts.
    robot_description = ParameterValue(Command([
        "xacro ", model,
        " use_real_hardware:=true",
        " dry_run:=", dry_run,
        " enabled_groups:=", enabled_groups,
        " mrl_host:=", LaunchConfiguration("mrl_host"),
        " mrl_port:=", LaunchConfiguration("mrl_port"),
        " hold_refresh_s:=", LaunchConfiguration("hold_refresh_s"),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument(
            "enabled_groups",
            default_value="head,rightArm,leftArm,rightHand,leftHand,torso"),
        DeclareLaunchArgument("mrl_host", default_value="pi5-inmoov.local"),
        DeclareLaunchArgument("mrl_port", default_value="8888"),
        DeclareLaunchArgument(
            "hold_refresh_s", default_value="0.0",
            description="Seconds between re-asserting an UNCHANGED pose. 0 = never "
                        "(default; holding torque cooks these servos)."),
        DeclareLaunchArgument("rviz", default_value="true"),

        LogInfo(msg=["[inmoov] ros2_control  dry_run=", dry_run,
                     "  groups=", enabled_groups]),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="both", parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node", output="both",
             parameters=[{"robot_description": robot_description},
                         PathJoinSubstitution(
                             [bringup_pkg, "config", "inmoov_controllers.yaml"])]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["inmoov_position_controller", "-c", "/controller_manager"]),

        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_state_publisher_gui",
             remappings=[("joint_states", "slider_joint_states")]),
        Node(package="inmoov_bringup", executable="slider_to_controller.py",
             name="slider_to_controller", output="screen",
             parameters=[{"joints": _controller_joints()}]),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution(
                 [description_pkg, "config", "inmoov.rviz"])]),
    ])
