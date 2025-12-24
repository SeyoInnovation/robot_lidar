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

    # 声明 launch 参数
    lidar_port_arg = DeclareLaunchArgument(
        name='lidar_port',
        default_value='/dev/ttyACM0',
        description='LIDAR serial port (e.g., /dev/ttyACM0)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

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

    # 2. 静态 TF: base_link → base_laser (雷达相对于机器人中心)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.07', '0.0', '0.05', '0', '0', '0', 'base_link', 'base_laser'],
        output='screen'
    )

    # 3. 静态 TF: base_link → base_footprint (通常在地面下方)
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
            launch_arguments={
            'params_file': ldlidar_config   # 关键：把配置文件传进去
            }.items()
        )

    # 5. slam_toolbox 在线异步建图
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file
        }.items()
    )

    # ==================== 组装 LaunchDescription ====================
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(lidar_port_arg)
    ld.add_action(use_rviz_arg)

    # 添加所有节点和子 launch
    ld.add_action(serial_port_node)
    ld.add_action(odometry_node)
    ld.add_action(tf_base_to_laser)
    ld.add_action(tf_base_to_footprint)
    ld.add_action(ldlidar_start)
    ld.add_action(slam_launch)

    return ld