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
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
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
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # AMCL localization
    amcl_config_file = os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml')
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('scan', '/scan')]
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz', 'bee1_navigation.rviz')
    
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
        map_file_arg,
        use_rviz_arg,
        bringup_launch,
        map_server_node,
        lifecycle_manager_node,
        amcl_node,
        rviz_node
    ])