import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    package_name = 'lidar' # 设置包名
    urdf_name = "robot.urdf"
    # # 配置路径
    # pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # urdf_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    # 包路径
    pkg_share = get_package_share_directory('lidar')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 发布 robot_description
    robot_description_content = Command(['xacro',' ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # 机器人状态发布器，发布出话题
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    # 加载场景
    gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={'gz_args': '-r src/lidar/world/demo.sdf'}.items()
        )
    
    # Spawn 模型 加载机器人
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='robot',
        arguments=[
            '-name', 'robot',  # 您的模型名
            '-topic', 'robot_description',  # 从 topic 加载 URDF
            '-x', '0.0', '-y', '0.0', '-z', '0.5'  # 位置
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', 'config_file:=src/lidar/config/bridge.yaml'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('world', default_value='empty', description='World file'),
        robot_state_publisher,
        gazebo_server,
        spawn,
        bridge
    ])