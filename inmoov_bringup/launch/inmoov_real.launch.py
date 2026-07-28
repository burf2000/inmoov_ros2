"""Bring up the REAL InMoov via ros2_control.

    ros2 launch inmoov_bringup inmoov_real.launch.py dry_run:=true

Starts controller_manager with the inmoov_hardware SystemInterface, the joint state
broadcaster and a group position controller, plus RViz. NO Gazebo: when the real robot is
the plant there is nothing to simulate.

This is the CONTROLLER path. If you want the RViz joint_state_publisher_gui sliders to
move the robot instead, use inmoov_sliders.launch.py -- those sliders publish straight to
/joint_states and never touch a controller.

SAFETY: exactly one thing may drive MyRobotLab at a time. Before running this, stop
inmoov-driver.service and inmoov-quest-bridge.service on pi5-inmoov, and start
inmoov-driver.service again when you are done.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("inmoov_description")
    bringup_pkg = FindPackageShare("inmoov_bringup")

    use_real_hardware = LaunchConfiguration("use_real_hardware")
    dry_run = LaunchConfiguration("dry_run")
    enabled_groups = LaunchConfiguration("enabled_groups")
    mrl_host = LaunchConfiguration("mrl_host")
    mrl_port = LaunchConfiguration("mrl_port")
    use_rviz = LaunchConfiguration("rviz")
    controllers = LaunchConfiguration("controllers_file")

    model = PathJoinSubstitution([description_pkg, "robots", "inmoov.urdf.xacro"])

    # The xacro args are what decide whether the URDF carries the REAL hardware
    # <ros2_control> block or the Gazebo one.
    # ParameterValue(..., value_type=str) is mandatory: without it launch tries to parse
    # the whole URDF as YAML and dies before anything starts.
    robot_description = ParameterValue(Command([
        "xacro ", model,
        " use_real_hardware:=", use_real_hardware,
        " dry_run:=", dry_run,
        " enabled_groups:=", enabled_groups,
        " mrl_host:=", mrl_host,
        " mrl_port:=", mrl_port,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_real_hardware", default_value="true",
            description="true = drive the physical robot through MRL. This launch file "
                        "defaults to true; it is the whole point of it."),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = log every servo move the driver WOULD make and send "
                        "nothing to MRL. Always prove a new setup here first."),
        DeclareLaunchArgument(
            "enabled_groups", default_value="head",
            description="Comma-separated bring-up groups actually driven: head, eyes, "
                        "rightArm, leftArm, rightHand, leftHand, torso. Default head only "
                        "-- every other group's direction signs are unverified."),
        DeclareLaunchArgument("mrl_host", default_value="pi5-inmoov.local"),
        DeclareLaunchArgument("mrl_port", default_value="8888"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [bringup_pkg, "config", "inmoov_controllers.yaml"])),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[{"robot_description": robot_description}, controllers],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager",
                       "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["inmoov_position_controller", "--controller-manager",
                       "/controller_manager"],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(use_rviz),
            arguments=["-d", PathJoinSubstitution(
                [description_pkg, "config", "inmoov.rviz"])],
        ),
    ])
