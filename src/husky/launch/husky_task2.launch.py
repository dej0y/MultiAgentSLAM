#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('camp')

    # Path to Task2.world
    world_file = PathJoinSubstitution([
        FindPackageShare('camp'),
        'worlds',
        'Task2.world'
    ])

    # Path to the Husky URDF file
    urdf_file = os.path.join(pkg_dir, 'models', 'husky_robot_model', 'model.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Robot description parameter
    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn Husky in Gazebo at position (-7, 0, 5)
    spawn_husky = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'husky', '-topic', 'robot_description', '-x', '-7', '-y', '5', '-z', '0.1'],
        output='screen'
    )

    # Gazebo launch (classic Gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_husky
    ])
