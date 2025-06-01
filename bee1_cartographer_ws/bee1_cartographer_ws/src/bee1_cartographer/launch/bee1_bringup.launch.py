#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    pkg_dir = FindPackageShare(package='bee1_cartographer').find('bee1_cartographer')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    # Robot description using xacro
    urdf_file = os.path.join(pkg_dir, 'urdf', 'bee1.urdf.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description_content}
        ]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Robot Localization EKF
    ekf_config_file = os.path.join(pkg_dir, 'config', 'robot_localization', 'ekf.yaml')
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Static transform publishers for sensor frames
    # LiDAR transform
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne_link', 'velodyne'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # GPS transform
    gps_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'gps_link', 'gps'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Velodyne LiDAR driver
    velodyne_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        output='screen',
        parameters=[
            {'device_ip': '192.168.1.201'},
            {'port': 2368},
            {'model': 'VLP16'},
            {'rpm': 600.0},
            {'frame_id': 'velodyne_link'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    velodyne_convert_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform',
        output='screen',
        parameters=[
            {'model': 'VLP16'},
            {'calibration': os.path.join(pkg_dir, 'config', 'VLP16db.yaml')},
            {'frame_id': 'velodyne_link'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # PointCloud to LaserScan converter
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'target_frame': 'base_link'},
            {'transform_tolerance': 0.01},
            {'min_height': -0.5},
            {'max_height': 2.0},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.0087},
            {'scan_time': 0.1},
            {'range_min': 0.1},
            {'range_max': 100.0},
            {'use_inf': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cloud_in', '/velodyne_points'),
            ('scan', '/scan')
        ]
    )
    
    # Custom nodes
    geojson_parser_node = Node(
        package='bee1_cartographer',
        executable='geojson_parser',
        name='geojson_parser',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    mission_executor_node = Node(
        package='bee1_cartographer',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'auto_start': False}
        ]
    )
    
    vehicle_controller_node = Node(
        package='bee1_cartographer',
        executable='vehicle_controller',
        name='vehicle_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'max_speed': 15.0},
            {'max_steering_angle': 0.5}
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
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        robot_localization_node,
        navsat_transform_node,
        lidar_tf_node,
        gps_tf_node,
        velodyne_node,
        velodyne_convert_node,
        pointcloud_to_laserscan_node,
        geojson_parser_node,
        mission_executor_node,
        vehicle_controller_node,
        rviz_node
    ])