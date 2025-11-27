from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',  # mapping 或 localization
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
                # 可选调参
                'solver_plugin': 'solver_plugins::CeresSolver',
                'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_preconditioner': 'SCHUR_JACOBI',
                'ceres_summary': True,
                'ceres_max_iterations': 20,
            }]
        ),
    ])