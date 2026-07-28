"""RViz + joint_state_publisher_gui sliders, optionally driving the REAL InMoov.

    # look only, nothing moves (identical to inmoov_description display.launch.py)
    ros2 launch inmoov_bringup inmoov_sliders.launch.py

    # prove the pipeline: sliders logged as servo degrees, nothing sent
    ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true dry_run:=true

    # for real
    ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true

This is the path Simon actually asked for: dragging a slider in the
joint_state_publisher_gui moves the robot. Those sliders publish straight to
/joint_states, which no controller ever sees, so ros2_control is NOT involved here at all.
The joint_state_relay node subscribes to /joint_states and drives MRL using the very same
mapping table and the very same code as the ros2_control hardware interface.

Note the URDF is loaded with use_real_hardware:=false on purpose: this path does not want
a <ros2_control> block of either flavour, because there is no controller_manager running.

SAFETY: exactly one thing may drive MyRobotLab at a time. Stop inmoov-driver.service and
inmoov-quest-bridge.service on pi5-inmoov first, and restart inmoov-driver.service after.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("inmoov_description")

    use_gui = LaunchConfiguration("use_gui")
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    dry_run = LaunchConfiguration("dry_run")
    enabled_groups = LaunchConfiguration("enabled_groups")
    mrl_host = LaunchConfiguration("mrl_host")
    mrl_port = LaunchConfiguration("mrl_port")
    use_rviz = LaunchConfiguration("rviz")

    model = PathJoinSubstitution([description_pkg, "robots", "inmoov.urdf.xacro"])
    robot_description = Command(["xacro ", model, " use_real_hardware:=false"])

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="joint_state_publisher_gui (the sliders)"),
        DeclareLaunchArgument(
            "use_real_hardware", default_value="false",
            description="true = relay /joint_states to the physical robot via MRL."),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = log what the relay WOULD send and send nothing."),
        DeclareLaunchArgument(
            "enabled_groups", default_value="head,rightArm,leftArm,rightHand,leftHand,torso",
            description="Groups actually driven. Default head only."),
        DeclareLaunchArgument("mrl_host", default_value="pi5-inmoov.local"),
        DeclareLaunchArgument("mrl_port", default_value="8888"),
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        # The bit that makes the sliders real.
        Node(
            package="inmoov_hardware",
            executable="joint_state_relay",
            name="inmoov_joint_state_relay",
            output="screen",
            condition=IfCondition(use_real_hardware),
            parameters=[{
                "enabled_groups": enabled_groups,
                # Launch substitutions are strings; the node declares these as bool/int,
                # so state the type explicitly or the parameter set is rejected at start.
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "mrl_host": mrl_host,
                "mrl_port": ParameterValue(mrl_port, value_type=int),
            }],
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
