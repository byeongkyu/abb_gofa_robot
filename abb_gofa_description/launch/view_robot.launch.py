import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot.urdf.xacro"),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':
                    Command([
                        'xacro',
                        ' ',
                        PathJoinSubstitution(
                            [get_package_share_directory('abb_gofa_description'),
                            'urdf',
                            LaunchConfiguration('robot_name')]
                        )
                    ]),
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        )

    ])