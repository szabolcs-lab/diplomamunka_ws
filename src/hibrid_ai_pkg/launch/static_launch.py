from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    package_dir = get_package_share_directory('hibrid_ai_pkg')
    simulation_resources_dir = get_package_share_directory('simulation_resources_pkg')
    simulation_resources_maps_dir = os.path.join(simulation_resources_dir, 'maps')
    
    map_file_arg = DeclareLaunchArgument('map_file', default_value='occupancy_grid_1.csv',)
    full_map_path = PathJoinSubstitution([simulation_resources_maps_dir, LaunchConfiguration('map_file')])
    
    map_publication_parameter_file = os.path.join(package_dir, 'configs', 'map_publication_params.yaml')
    path_planner_parameter_file = os.path.join(package_dir, 'configs', 'd_star_lite_path_planner_params.yaml')
    
    
    # 1) map -> OccupancyGrid /map
    map_publication = Node(
        package='hibrid_ai_pkg',
        executable='map_publication',
        name='map_publication',
        output='screen',
        parameters=[map_publication_parameter_file, {'map_file': full_map_path}]
    )
    
    # 2) D* Lite path planner -> /planned_path_dilated (frame: map)
    d_star_lite_path_planner = Node(
        package='hibrid_ai_pkg',
        executable='d_star_lite_path_planner',
        name='d_star_lite_path_planner',
        output='screen',
        parameters=[path_planner_parameter_file, {'map_file': full_map_path}, {'scenario': 'static'}], 
    )
    
    # 3) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    

     # 5) odom -> base_link TF (Odometry-ből)
    tf_broadcaster = Node(
        package='hibrid_ai_pkg',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        parameters=[{'odom_topic': '/odom'}]
    )

    # 6) ROS /cmd_vel -> Ignition /cmd_vel
    gz_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_cmd_vel_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist']
    )
    
    
    # 7) Ignition odometry -> ROS /odom
    gz_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=['/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/vehicle_blue/odometry', '/odom')],
        output='screen'
    )
    
    gz_bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/lidar', '/scan')],
        output='screen'
    )

    
    ppo_agent = Node(
        package='hibrid_ai_pkg',
        executable='ppo_agent',
        name='ppo_agent',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'chassis',
            'path_topic': '/planned_path_dilated',
            'scan_topic': '/scan',
            'cmd_topic': '/cmd_vel',
            'use_pure_pursuit_fallback': True
        }]
    )

    
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['-10', '10', '0', '0', '0', '0', 'map', 'odom']
    )

    
    return LaunchDescription([
        map_file_arg,
        map_publication,
        d_star_lite_path_planner,
        rviz,
        gz_cmd_vel_bridge,
        gz_bridge_odom,
        map_to_odom,
        tf_broadcaster,
        gz_bridge_lidar,
        ppo_agent
    ])