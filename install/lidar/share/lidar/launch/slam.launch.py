import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # 参数配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 包位置
    pkg_share = get_package_share_directory('lidar') 
    #rviz2参数配置
    rviz_path = os.path.join(pkg_share,'config','slam.rviz') 
    # 启动里程计
    odom = Node(
        package='lidar',          
            executable='odom',     
            name='fake_odom',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'linear_speed': 0.22},   
                {'angular_speed': 0.0}
            ]
    )
    # 启动雷达
    ldlidar_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ldlidar'),
                    'launch',
                    'ld14p.launch.py'  
                )
            )
        )
    
    # 调用slam_toolbox建图
    Slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        parameters=[
            {
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'minimum_travel_distance': 0.01,
                'minimum_travel_heading': 0.01,
                'transform_publish_period': 0.02,
                'resolution': 0.05,
            }
        ]
    )
    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
        )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        odom,
        ldlidar_start,
        # static_tf_laser,
        # static_tf_odom,
        Slam,
        start_rviz_cmd
    ])