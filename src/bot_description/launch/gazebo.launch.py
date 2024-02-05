#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    packages_name = "bot_description"
    xacro_file_name = "bot.urdf.xacro"

    # Generate URDF using xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {
        'use_sim_time': True,
        "robot_description": robot_description_content
    }
    
    state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # '-v', '4',
            '-entity', 'ylvov_robot',
            '-topic', 'robot_description',
            '-world', 'yura',
        ],
        output='screen'
    )

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('bot_description'), 'config', 'config.rviz')]
        )

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('bot_gazebo'),
        'worlds',
        'empty.world'
    ])

    launch_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', world_file_path)]
        )

    return LaunchDescription([
        state_publisher,
        # joint_state_pub_gui_node,
        rviz,
        # spawn_entity,
        launch_world,
        spawn,
    ])