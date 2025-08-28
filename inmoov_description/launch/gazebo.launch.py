#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from ament_index_python.packages import get_package_prefix
from launch.actions import TimerAction


def generate_launch_description():

    set_gz_sys = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        str(Path(get_package_prefix('gz_ros2_control')) / 'lib') + ':' +
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )

    # Resolve package share
    pkg_share = Path(get_package_share_directory('inmoov_description'))  # .../share/inmoov_description
    gz_resource_root = str(pkg_share.parent)                             # .../share

    # Add our share/ root to Gazebo resource paths (preserve user env if set)
    gz_res  = gz_resource_root + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ign_res = gz_resource_root + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    set_gz  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res)
    set_ign = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_res)  # legacy; harmless

    # Defaults
    pkg = FindPackageShare('inmoov_description')
    default_model = PathJoinSubstitution([pkg, 'robots', 'inmoov.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg, 'config', 'inmoov.rviz'])
    default_world = 'empty.sdf'  # built-in world

    use_gui = LaunchConfiguration('use_gui')
    model   = LaunchConfiguration('model')
    rviz    = LaunchConfiguration('rviz')
    rviz_cfg= LaunchConfiguration('rviz_config')
    world   = LaunchConfiguration('world')
    name    = LaunchConfiguration('name')

    # Xacro -> robot_description (keep a space after 'xacro ' with Command)
    robot_description = Command(['xacro ', model])

    # Also write URDF to a file for the gz spawner
    urdf_out = '/tmp/inmoov.urdf'
    xacro_to_file = ExecuteProcess(cmd=['xacro', model, '-o', urdf_out], output='screen')

    # Start Gazebo (gz-sim)
    gz = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # State publishers + RViz
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    )

    # Spawn robot into gz-sim
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_inmoov',
        arguments=['-name', name, '-file', urdf_out],
        output='screen'
    )

    # Spawn controllers (Jazzy)
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        arguments=['joint_state_broadcaster',
                '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawner_pos = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_inmoov_position',
        arguments=['inmoov_position_controller',
                '--controller-manager', '/controller_manager'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

   
    # Order: xacro -> start gz -> spawn robot -> spawn controllers
    delayed_gz     = TimerAction(period=5.5, actions=[gz])
    delayed_spawn  = TimerAction(period=2.0, actions=[spawn])
    delayed_ctrls  = TimerAction(period=6.0, actions=[spawner_jsb, spawner_pos])


    # Start it shortly after gz comes up
    delayed_clock = TimerAction(period=1.0, actions=[clock_bridge])
    
    return LaunchDescription([
        set_gz, set_ign,
        DeclareLaunchArgument('use_gui', default_value='true', description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('model', default_value=default_model, description='Path to inmoov Xacro'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz2 config'),
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world name or SDF path'),
        DeclareLaunchArgument('name',  default_value='inmoov', description='Entity name in Gazebo'),
        set_gz_sys,
        xacro_to_file,
        delayed_gz,
        jsp_gui, rsp, rviz2, #jsp_headless
        delayed_spawn,
        delayed_ctrls,
        delayed_clock
    ])
