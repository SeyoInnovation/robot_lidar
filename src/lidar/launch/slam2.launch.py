#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        # 1. 假里程计（你原来的）
        Node(
            package='lidar',
            executable='odom',
            name='fake_odom',
            output='screen'
        ),

        # 2. 雷达（保持你原来能正常开的那个）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ld14p.launch.py')
            )
        ),

        # 3. slam_toolbox —— 强制能跑通版（重点在这段！）
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[
                ('scan', '/scan'),      # 强制认话题
                ('odom', '/odom'),
                ('map', '/slam_map'),
            ],
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',                 # 重复写一遍保险
                'minimum_travel_distance': 0.005,      # 0.5cm 就触发
                'minimum_travel_heading': 0.005,
                'transform_publish_period': 0.02,      # 强制 50Hz 发 map→odom
                'map_update_interval': 1.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
            }]
        ),

        # 4. RViz（可选）
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])