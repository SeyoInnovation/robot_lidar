#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==================== 参数和包路径 ====================
    package_name = 'lidar'  # 你的包名
    my_pkg_dir = get_package_share_directory(package_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # 导航配置文件路径
    map_file = os.path.join(os.path.expanduser('~'), 'robot_lidar', 'my_room.yaml')
    params_file = os.path.join(my_pkg_dir, 'config', 'my_nav2_params.yaml')

    # ==================== 节点和 launch 包含 ====================

    # 1. 你的串口读取 + 数据处理节点（假设你有两个可执行文件）
    serial_port_node = Node(
        package=package_name,
        executable='serial_read',  # 你的串口读取节点
        name='serial_read_node',
        output='screen'
    )

    odometry_node = Node(
        package=package_name,
        executable='odometry_pub',  # 你的里程计发布节点
        name='odometry_publisher',
        output='screen'
    )

    # 2.TF: base_link → base_laser (雷达相对于机器人中心)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.07', '0.0', '0.05', '0', '0', '0', 'base_link', 'base_laser'],
        output='screen'
    )

    # 3. TF: base_link → base_footprint (通常在地面下方)
    tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_footprint',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'base_link', 'base_footprint'],  # z 建议负值
        output='screen'
    )

    # 4. LD14P 雷达驱动 launch 文件
    ldlidar_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ldlidar'),
                    'launch',
                    'ld14p.launch.py'  
                )
            )
    )

    # 5. Nav2 启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'false', 
            'autostart': 'true'
        }.items()
    )

    # ==================== 组装 LaunchDescription ====================
    ld = LaunchDescription()

    # 添加所有节点和子 launch
    ld.add_action(serial_port_node)
    ld.add_action(odometry_node)
    ld.add_action(tf_base_to_laser)
    ld.add_action(tf_base_to_footprint)
    ld.add_action(ldlidar_start)
    ld.add_action(nav2_launch)

    return ld