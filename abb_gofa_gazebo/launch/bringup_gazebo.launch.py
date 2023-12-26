import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    robot_name = DeclareLaunchArgument("robot_name", default_value="gofa_robot")
    world_name = DeclareLaunchArgument("world_name", default_value="default.sdf")
    tf_prefix = DeclareLaunchArgument("tf_prefix", default_value="")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    model_path = get_package_share_directory('abb_gofa_description')
    model_path = model_path[:len(model_path) - len('abb_gofa_description')]
    environ['IGN_GAZEBO_RESOURCE_PATH'] = model_path

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            "gz_args": [
                '-r -v2',
                ' ',
                LaunchConfiguration('world_name')
            ]
        }.items()
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('abb_gofa_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim' : 'true',
            'tf_prefix': LaunchConfiguration('tf_prefix')
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description'
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_name,
        world_name,
        tf_prefix,
        gz_bridge,
        upload_robot,
        gz_sim,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
    ])