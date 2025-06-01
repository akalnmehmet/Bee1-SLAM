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
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='System mode: mapping, localization, navigation'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'sample_track.yaml'),
        description='Full path to map yaml file to load'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start mission execution'
    )
    
    # Include hardware launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bee1_cartographer'),
                'launch',
                'bee1_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
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
    
    # Include navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bee1_cartographer'),
                'launch',
                'bee1_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'use_rviz': 'true'
        }.items()
    )
    
    # System Monitor
    system_monitor_node = Node(
        package='bee1_cartographer',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'bee1_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        map_file_arg,
        auto_start_arg,
        hardware_launch,
        bringup_launch,
        navigation_launch,
        system_monitor_node
    ])