from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('rrt_star_pkg')

    nav2_controller_params = os.path.join(pkg_dir, 'configs', 'nav2_controller_params.yaml')

    # 1) Path -> FollowPath kliens
    nav2_path_client = Node(
        package='rrt_star_pkg',
        executable='nav2_path',
        name='nav2_path_client',
        output='screen',
        parameters=[{
            'path_topic': 'planned_path_dilated'
        }]
    )

    # 2) Nav2 controller_server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_controller_params],
        arguments=['--ros-args', '--log-level', 'controller_server:=debug']
    )

    # 3) lifecycle manager csak a controllerre
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server']
        }]
    )
    
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['-10', '10', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        nav2_path_client,
        controller_server,
        lifecycle_manager,
        static_map_to_odom
    ])