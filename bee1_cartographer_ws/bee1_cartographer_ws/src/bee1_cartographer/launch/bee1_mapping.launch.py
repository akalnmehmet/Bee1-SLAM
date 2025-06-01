#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_dir = FindPackageShare(package='bee1_cartographer').find('bee1_cartographer')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(pkg_dir, 'config', 'cartographer'),
        description='Full path to config file to load'
    )
    
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='bee1_2d.lua',
        description='Name of lua file for cartographer'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'
    )
    
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'
    )
    
    # Include bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bee1_cartographer'),
                'launch',
                'bee1_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': 'false'
        }.items()
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('configuration_basename')
        ],
        remappings=[
            ('points2', '/velodyne_points'),
            ('imu', '/imu/data'),
            ('fix', '/gps/fix')
        ]
    )
    
    # Cartographer occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': LaunchConfiguration('resolution')},
            {'publish_period_sec': LaunchConfiguration('publish_period_sec')}
        ]
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz', 'bee1_mapping.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        configuration_directory_arg,
        configuration_basename_arg,
        use_rviz_arg,
        resolution_arg,
        publish_period_sec_arg,
        bringup_launch,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node
    ])