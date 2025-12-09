import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'lidar' # 设置包名
    # 配置路径
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # 节点写明
    serial_port_node = Node(
        package='lidar',
        executable='serial_read',
        name='open_serialport_recieve&post'
        )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    ld.add_action(serial_port_node)
    


    return ld
