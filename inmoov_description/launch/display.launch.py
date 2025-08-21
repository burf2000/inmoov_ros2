# inmoov_description/launch/display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('inmoov_description')

    # Default model path – change to 'urdf/inmoov.urdf.xacro' if that's where yours lives
    default_model = PathJoinSubstitution([pkg, 'robots', 'inmoov.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg, 'config', 'inmoov.rviz'])

    use_gui = LaunchConfiguration('use_gui')
    model   = LaunchConfiguration('model')
    rviz    = LaunchConfiguration('rviz')
    rviz_cfg= LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('model', default_value=default_model,
                              description='Path to inmoov Xacro'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz2 config'),

        # GUI when use_gui==true
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        # Headless when use_gui==false
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro', ' ', model])}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz),
        ),
    ])
