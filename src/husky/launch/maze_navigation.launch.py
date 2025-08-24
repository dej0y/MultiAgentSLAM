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
    
    # Path to maze world file
    world_file = PathJoinSubstitution([
        FindPackageShare('camp'),
        'worlds',
        'Task2.world'
    ])
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'models', 'husky_robot_model', 'model.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Robot description parameter
    robot_description = {'robot_description': robot_description_content}

    # Gazebo launch with model path (classic Gazebo)
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

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn robot in Gazebo at a specific location in the maze (classic Gazebo)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'husky_robot',
            '-x', '-7.0',
            '-y', '5.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity
    ])


# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():
#     launch_file_dir = os.path.join(get_package_share_directory('camp'), 'launch')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='-7.0')
#     y_pose = LaunchConfiguration('y_pose', default='5.0')

#     world = os.path.join(
#         get_package_share_directory('camp'),
#         'worlds',
#         'Task2.world'
#     )

#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     robot_state_publisher_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
#         ),
#         launch_arguments={'use_sim_time': use_sim_time}.items()
#     )

#     spawn_turtlebot_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
#         ),
#         launch_arguments={
#             'x_pose': x_pose,
#             'y_pose': y_pose
#         }.items()
#     )

#     ld = LaunchDescription()

#     # Add the commands to the launch description
#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_turtlebot_cmd)

#     return ld
