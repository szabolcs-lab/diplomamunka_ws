from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    package_dir = get_package_share_directory('d_star_lite_pkg')
    simulation_resources_dir = get_package_share_directory('simulation_resources_pkg')
    simulation_resources_maps_dir = os.path.join(simulation_resources_dir, 'maps')
    map_file_arg = DeclareLaunchArgument('map_file', default_value='occupancy_grid_1.csv')
    full_map_path = PathJoinSubstitution([simulation_resources_maps_dir, LaunchConfiguration('map_file')])
    
    map_publication_parameter_file = os.path.join(package_dir, 'configs', 'map_publication_params.yaml')
    path_planner_parameter_file = os.path.join(package_dir, 'configs', 'd_star_lite_path_planner_params.yaml')
    nav2_bringup_launch = os.path.join(package_dir, 'launch', 'nav2_bringup.launch.py')

    # dynamic_obstacles_pkg launch (spawner + map_updater)
    dynamic_obstacles_pkg_dir = get_package_share_directory('dynamic_obstacles_pkg')
    dynamic_obstacles_launch = os.path.join(dynamic_obstacles_pkg_dir, 'launch', 'launch.py')

    
    # 1) map -> OccupancyGrid /map  (statikus map, mint eddig)
    map_publication = Node(
        package='d_star_lite_pkg',
        executable='map_publication',
        name='map_publication',
        output='screen',
        parameters=[map_publication_parameter_file, {'map_file': full_map_path}]
    )
    
    # 2) Dinamikus akadály miatt
    dynamic_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dynamic_obstacles_launch)
    )
    
    # 3) D* Lite path planner -> /planned_path_dilated (frame: map)
    #    Itt jön a lényeg: a node kódja 'map'-ra iratkozik fel,
    #    de mi átremappeljük neki 'map_dynamic'-ra.
    d_star_lite_path_planner = Node(
        package='d_star_lite_pkg',
        executable='d_star_lite_path_planner',
        name='d_star_lite_path_planner',
        output='screen',
        parameters=[path_planner_parameter_file, {'map_file': full_map_path}, {'scenario': 'dynamic'}],
        remappings=[('map', 'map_dynamic'),]
    )
    
    # 4) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # 5) Nav2 bringup (controller_server + lifecycle + static map->odom)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch)
    )
    
    # 6) odom -> base_link TF (Odometry-ből)
    tf_broadcaster = Node(
        package='d_star_lite_pkg',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        parameters=[{'odom_topic': '/odom'}]
    )
    
    # 7) ROS /cmd_vel -> Ignition /cmd_vel
    gz_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_cmd_vel_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist']
    )
    
    # 8) Ignition odometry -> ROS /odom
    gz_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=['/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/vehicle_blue/odometry', '/odom')],
        output='screen'
    )
    
    return LaunchDescription([
        map_file_arg,
        map_publication,
        dynamic_obstacles,          
        d_star_lite_path_planner,   
        rviz,
        nav2_bringup,
        gz_cmd_vel_bridge,
        gz_bridge_odom,
        tf_broadcaster,
    ])
