#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    number_of_robots = 4
    namespace = 'BOT'   
    pose = [[-3.0, -1.0], [-2.0, 4.0], [1.5, -3.0], [4.0, 1.7]]
    dynamic_logistics_warehouse_pkg = get_package_share_directory('dynamic_logistics_warehouse')
    model_folder = os.path.join(dynamic_logistics_warehouse_pkg, 'models', 'turtlebot3_burger')
    urdf_path = os.path.join(model_folder, 'model.sdf')

    save_path = os.path.join(model_folder, 'tmp')
    
    launch_file_dir = os.path.join(get_package_share_directory('dynamic_logistics_warehouse'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    dynamic_logistics_warehouse_pkg = get_package_share_directory('dynamic_logistics_warehouse')


    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    world = os.path.join(dynamic_logistics_warehouse_pkg, 'worlds', 'warehouse.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    rviz_cmd = GroupAction(
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz',
                    arguments=['-d', os.path.join(dynamic_logistics_warehouse_pkg, 'rviz', 'multibot.rviz')],
                )
            ],
        )

    static_tf_publishers = []
    robot_state_publisher_cmd_list = []

    for count in range(number_of_robots):
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace}_{count+1}'
                    }.items()
            )
        )
        static_tf_publishers.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_bot_{count+1}',
                arguments=['0', '0', '0', '0', '0', '0', 'world', f'{namespace}_{count+1}/odom']
            )
        )

    spawn_turtlebot_cmd_list = []

    for count in range(number_of_robots):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}_{count+1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}_{count+1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}_{count+1}/base_scan'
        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
        with open(f'{save_path}{count+1}.sdf', 'w') as file:
            file.write(urdf_modified)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'multi_spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': str(pose[count][0]),
                'y_pose': str(pose[count][1]),
                'z_pose': '0.01',
                'yaw': '0.0',
                'robot_name': f'{TURTLEBOT3_MODEL}_{count+1}',
                'namespace': f'{namespace}_{count+1}',
                'sdf_path': f'{save_path}{count+1}.sdf'
            }.items()
            )
        )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(rviz_cmd)   
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') for count in range(number_of_robots)]
        )
    ))
    for static_tf in static_tf_publishers:
        ld.add_action(static_tf)

    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([PushRosNamespace(f'{namespace}_{count}'),
                                  robot_state_publisher_cmd_list[count-1],
                                  spawn_turtlebot_cmd]))

    return ld
