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

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('inmoov_description'))             # .../install/inmoov_description/share/inmoov_description
    ign_resource_root = str(pkg_share.parent)                                       # .../install/inmoov_description/share

    # Allow users’ existing paths to stay in effect
    ign_res = ign_resource_root + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_res  = ign_resource_root + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    set_ign = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_res)
    set_gz  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res)

    pkg = FindPackageShare('inmoov_description')
    default_model = PathJoinSubstitution([pkg, 'robots', 'inmoov.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg, 'config', 'inmoov.rviz'])
    default_world = 'empty.sdf'

    use_gui = LaunchConfiguration('use_gui')
    model   = LaunchConfiguration('model')
    rviz    = LaunchConfiguration('rviz')
    rviz_cfg= LaunchConfiguration('rviz_config')
    world   = LaunchConfiguration('world')
    name    = LaunchConfiguration('name')

    # IMPORTANT: keep package:// URIs in xacros; Command needs a space after 'xacro '
    robot_description = Command(['xacro ', model])

    urdf_out = '/tmp/inmoov.urdf'
    xacro_to_file = ExecuteProcess(cmd=['xacro', model, '-o', urdf_out], output='screen')

    ign = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world], output='screen')

    jsp_gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                   name='joint_state_publisher_gui', condition=IfCondition(use_gui))
    jsp_headless = Node(package='joint_state_publisher', executable='joint_state_publisher',
                        name='joint_state_publisher', condition=UnlessCondition(use_gui))
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    rviz2 = Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', rviz_cfg], condition=IfCondition(rviz))

    spawn = Node(package='ros_ign_gazebo', executable='create', name='spawn_inmoov',
                 arguments=['-name', name, '-file', urdf_out], output='screen')

    delayed_ign   = TimerAction(period=0.5, actions=[ign])
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    return LaunchDescription([
        set_ign, set_gz,                                  # <— add these
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('name',  default_value='inmoov'),
        xacro_to_file,
        delayed_ign,
        jsp_gui, jsp_headless, rsp, rviz2,
        delayed_spawn,
    ])
