import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')          #是否启用仿真时钟

    #------------------------------------------------------------------
    # 包路径（所有路径都用pkg_share来索引）
    pkg_share = get_package_share_directory('lidar')        #项目包路径
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')      #gz包路径

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    # 桥接配置文件绝对路径
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    # 世界文件绝对路径
    world_path = os.path.join(pkg_share, 'world', 'demo.sdf')
    # 发布 robot_description
    robot_description_content = Command(['xacro',' ', urdf_path])
    robot_description = {
        'robot_description': robot_description_content
        }
    #rviz2参数配置
    rviz_path = os.path.join(pkg_share,'config','default.rviz')   

    #-----------------------------------------------------------------------

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
            PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
        )
    
    # Spawn 模型，加载机器人
    spawn = Node(
        package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file', os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),  # 直接喂 xacro！
        '-name', 'robot',
        '-z', '0.1'
    ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_config}
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
    # 数据转化
    cmd_vel_to_joint_states = Node(
            package='lidar',
            executable='vel_to_joint',
            name='cmd_vel_to_joint_states',
            output='screen',
            parameters=[
                {'wheel_separation': 0.20},
                {'wheel_diameter': 0.064},
                {'cmd_vel_topic': '/cmd_vel'},
            ]
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        # DeclareLaunchArgument('world', default_value='empty', description='World file'),
        robot_state_publisher,
        gazebo_server,
        spawn,
        bridge,
        cmd_vel_to_joint_states,
        start_rviz_cmd
    ])