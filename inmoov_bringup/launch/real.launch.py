"""THE ONE YOU WANT. RViz + sliders driving the REAL InMoov.

    ros2 launch inmoov_bringup real.launch.py                    # for real
    ros2 launch inmoov_bringup real.launch.py dry_run:=true      # log only, nothing moves

Sliders publish /joint_states; the joint_state_relay converts each joint to servo degrees
around that servo's calibrated REST and sends it to MyRobotLab. No controller_manager is
involved, which is why the URDF is loaded WITHOUT a <ros2_control> block.

Servos are commanded ONLY when a joint actually moves. Nothing re-asserts a held pose, so
a limb will sag rather than sit under continuous holding torque — deliberate: these servos
have no thermal protection and a sag is recoverable where a cooked servo is a rebuild.

SAFETY: exactly one thing may drive MyRobotLab at a time.
    ssh burf2000@pi5-inmoov.local 'sudo systemctl stop inmoov-driver'     # before
    ssh burf2000@pi5-inmoov.local 'sudo systemctl start inmoov-driver'    # after
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("inmoov_description")
    model = PathJoinSubstitution([pkg, "robots", "inmoov.urdf.xacro"])

    dry_run = LaunchConfiguration("dry_run")
    enabled_groups = LaunchConfiguration("enabled_groups")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = log every servo move that WOULD be sent and send nothing."),
        DeclareLaunchArgument(
            "enabled_groups",
            default_value="head,rightArm,leftArm,rightHand,leftHand,torso",
            description="Narrow this to stage bring-up, e.g. enabled_groups:=head,rightHand"),
        DeclareLaunchArgument("mrl_host", default_value="pi5-inmoov.local"),
        DeclareLaunchArgument("mrl_port", default_value="8888"),
        DeclareLaunchArgument(
            "hold_refresh_s", default_value="0.0",
            description="Seconds between re-asserting an UNCHANGED pose. 0 = never (default; "
                        "holding torque cooks these servos). 0.5 holds a pose against gravity."),
        DeclareLaunchArgument("rviz", default_value="true"),

        LogInfo(msg=["[inmoov] REAL HARDWARE  dry_run=", dry_run,
                     "  groups=", enabled_groups]),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(
                 Command(["xacro ", model, " use_real_hardware:=false"]), value_type=str)}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_state_publisher_gui"),

        Node(
            package="inmoov_hardware", executable="joint_state_relay",
            name="inmoov_joint_state_relay", output="screen",
            parameters=[{
                "enabled_groups": enabled_groups,
                # Launch substitutions are strings; the node declares these as bool/int/double,
                # so the type must be stated or the parameter set is rejected at start-up.
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "mrl_host": LaunchConfiguration("mrl_host"),
                "mrl_port": ParameterValue(LaunchConfiguration("mrl_port"), value_type=int),
                "hold_refresh_s": ParameterValue(
                    LaunchConfiguration("hold_refresh_s"), value_type=float),
            }],
        ),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution([pkg, "config", "inmoov.rviz"])]),
    ])
