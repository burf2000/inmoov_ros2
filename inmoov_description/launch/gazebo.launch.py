#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path
import os


def generate_launch_description():

    # --- Paths & env so Gazebo can find your meshes and the gz_ros2_control plugin ---
    set_gz_sys = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        str(Path(get_package_prefix('gz_ros2_control')) / 'lib') + ':' +
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )

    pkg_share = Path(get_package_share_directory('inmoov_description'))  # .../share/inmoov_description
    gz_resource_root = str(pkg_share.parent)                             # .../share

    set_gz  = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_root + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    set_ign = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        gz_resource_root + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # --- Defaults (use the FULL robot xacro that defines links/joints) ---
    pkg = FindPackageShare('inmoov_description')
    default_model = PathJoinSubstitution([pkg, 'robots', 'inmoov.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg, 'config', 'inmoov.rviz'])
    default_world = 'empty.sdf'  # built-in world

    use_gui = LaunchConfiguration('use_gui')   # joint_state_publisher_gui (off by default)
    model   = LaunchConfiguration('model')     # xacro path (full robot)
    rviz    = LaunchConfiguration('rviz')
    rviz_cfg= LaunchConfiguration('rviz_config')
    world   = LaunchConfiguration('world')
    name    = LaunchConfiguration('name')

    # --- Build URDF once; Gazebo will spawn this file (race-proof: we wait below) ---
    urdf_out = '/tmp/inmoov.urdf'
    xacro_to_file = ExecuteProcess(
        cmd=['xacro', model, '-o', urdf_out],
        output='screen'
    )

    # --- Gazebo (gz sim) ---
    gz = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # --- Clock bridge (start ONCE and EARLY; correct mapping) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # --- Robot State Publisher uses SAME model description (no drift) ---
    # IMPORTANT: keep a space after 'xacro ' so it doesn't become 'xacro/home/...'
    robot_description = Command(['xacro ', model])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # --- Optional GUI joint publisher (OFF for sim; it can fight real /joint_states) ---
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    )

    # --- RViz ---
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
        output='screen'
    )

    # --- Spawn robot into Gazebo AFTER xacro_to_file completes ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_inmoov',
        arguments=['-name', name, '-file', urdf_out],
        output='screen'
    )

    # --- Controllers (Controller Manager at /controller_manager) ---
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    spawner_pos = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_inmoov_position',
        arguments=['inmoov_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # --- Ordering: Gazebo soon; spawn waits for xacro; controllers+RViz wait for spawn ---
    delayed_gz = TimerAction(period=0.5, actions=[gz])

    spawn_after_xacro = RegisterEventHandler(
        OnProcessExit(target_action=xacro_to_file, on_exit=[spawn])
    )
    ctrls_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawner_jsb, spawner_pos, rviz2])
    )

    delayed_clock = TimerAction(period=1.0, actions=[clock_bridge])

    return LaunchDescription([
        set_gz, set_ign, set_gz_sys,
        DeclareLaunchArgument('use_gui', default_value='false', description='Use joint_state_publisher_gui (off for sim)'),
        DeclareLaunchArgument('model', default_value=default_model, description='Path to full InMoov xacro (robots/inmoov.urdf.xacro)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz2 config'),
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world name or SDF path'),
        DeclareLaunchArgument('name',  default_value='inmoov', description='Entity name in Gazebo'),

        xacro_to_file,
        delayed_clock,     
        delayed_gz,
        rsp,                # parses same model; uses sim time
        jsp_gui,            # usually disabled (use_gui:=false)

        # wait chains
        spawn_after_xacro,  # spawn after URDF written
        ctrls_after_spawn,  # controllers + RViz after spawn
    ])
