import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, PythonExpression

def generate_launch_description():
    tf_prefix = DeclareLaunchArgument("tf_prefix", default_value="")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('abb_gofa_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' ',
                    "tf_prefix:=", LaunchConfiguration('tf_prefix')
                ]),
        }]
    )

    return LaunchDescription([
        tf_prefix,
        rsp_node
    ])