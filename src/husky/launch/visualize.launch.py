#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('camp')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'models', 'husky_robot_model', 'model.urdf')
    sdf_file = os.path.join(pkg_dir, 'models', 'husky_robot_model', 'model.sdf')
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

    # Custom joint state publisher - publishes default joint states reliably
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )

    joint_state_publisher_node = Node(
        package='camp',
        executable='simple_joint_state_publisher.py',
        name='simple_joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # RViz2 node with configuration
    rviz_config_file = os.path.join(pkg_dir, 'config', 'husky_visualization.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    # Gazebo launch with custom world
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    camp_pkg_share = get_package_share_directory('camp')
    world_path = os.path.join(camp_pkg_share, 'worlds', 'Task.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn robot in Gazebo using URDF file
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'husky',
            # '-file', urdf_file,
            # '-file', sdf_file,
            '-topic', 'robot_description',
            '-x', '-7.0',
            '-y', '5.0',
            '-z', '0.0',
            '-Y', '3.14'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        spawn_entity
    ])
