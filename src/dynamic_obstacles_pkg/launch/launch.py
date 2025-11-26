from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('dynamic_obstacles_pkg')
    params_file_dynamic_obstacle = os.path.join(pkg_share, 'configs', 'dynamic_obstacles_params.yaml')
    params_file_dynamic_obstacle_to_map_updater = os.path.join(pkg_share, 'configs', 'dynamic_obstacle_to_map_update_params.yaml')
    
    return LaunchDescription([
        # Dinamikus akadály spawner
        Node(
            package='dynamic_obstacles_pkg',
            executable='dynamic_obstacle',
            name='dynamic_obstacle_spawner',
            output='screen',
            parameters=[params_file_dynamic_obstacle],
        ),

        # Dinamikus map updatelő
        Node(
            package='dynamic_obstacles_pkg',
            executable='dynamic_obstacle_to_map_update',
            name='dynamic_map_updater',
            output='screen',
            parameters=[params_file_dynamic_obstacle_to_map_updater],
        ),
    ])
