#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    
    # Hardware Interface Node
    hardware_interface_node = Node(
        package='bee1_cartographer',
        executable='hardware_interface',
        name='hardware_interface',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'bee1_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # UBLOX GPS Driver
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[
            {'device': '/dev/gps'},
            {'frame_id': 'gps_link'},
            {'baudrate': 115200},
            {'rate': 10},
            {'enable_sbas': True},
            {'enable_bds': True},
            {'enable_gal': True},
            {'enable_glo': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # XSENS IMU Driver
    xsens_imu_node = Node(
        package='xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[
            {'device': '/dev/imu'},
            {'baudrate': 115200},
            {'frame_id': 'imu_link'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Velodyne VLP16 Driver
    velodyne_driver_node = Node(
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
    
    # Velodyne Point Cloud Converter
    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform',
        output='screen',
        parameters=[
            {'model': 'VLP16'},
            {'calibration': os.path.join(pkg_dir, 'config', 'VLP16db.yaml')},
            {'frame_id': 'velodyne_link'},
            {'organize_cloud': False},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        hardware_interface_node,
        ublox_gps_node,
        xsens_imu_node,
        velodyne_driver_node,
        velodyne_pointcloud_node
    ])