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

    map_file_arg = DeclareLaunchArgument('map_file', default_value='occupancy_grid_1.csv')
    full_map_path = PathJoinSubstitution([simulation_resources_maps_dir, LaunchConfiguration('map_file')])

    map_publication_parameter_file = os.path.join(package_dir, 'configs', 'map_publication_params.yaml')
    path_planner_parameter_file = os.path.join(package_dir, 'configs', 'd_star_lite_path_planner_params.yaml')
    nav2_bringup_launch = os.path.join(package_dir, 'launch', 'nav2_bringup.launch.py')

    # 1) map -> /map
    map_publication = Node(
        package='hibrid_ai_pkg',
        executable='map_publication',
        name='map_publication',
        output='screen',
        parameters=[map_publication_parameter_file, {'map_file': full_map_path}]
    )

    # 2) D* Lite -> /planned_path_dilated
    d_star_lite_path_planner = Node(
        package='hibrid_ai_pkg',
        executable='d_star_lite_path_planner',
        name='d_star_lite_path_planner',
        output='screen',
        parameters=[path_planner_parameter_file, {'map_file': full_map_path}, {'scenario': 'static'}],
    )

    # 3) Path finomító: /planned_path_dilated -> /planned_path_refined
    trajectory_smoother = Node(
        package='hibrid_ai_pkg',
        executable='trajectory_smoother',
        name='trajectory_smoother',
        output='screen',
        parameters=[{
            'path_in': '/planned_path_dilated',
            'path_out': '/planned_path_smoother',
            'params_topic': '/smoother_params'
        }]
    )

    # 4) PPO trainer
    # 4) PPO product (csak betölt és publikál)
    ppo_product = Node(
        package='hibrid_ai_pkg',
        executable='ppo_product',
        name='ppo_product',
        output='screen',
        parameters=[{
            'train_mode': False,           # PRODUCT: ne tanítson
            'control_hz': 10.0,

            # done feltételek productban nem létfontosságúak, de maradhatnak
            'max_steps': 1500, #1400
            'goal_tolerance': 0.8,
            'collision_distance': 0.18,
            'min_steps_for_goal': 50,

            'lidar_bins': 12,
            'lidar_max_range': 6.0,

            'max_offset_m': 0.10,
            'offset_limit': 0.05,
            'smooth_max': 0.25,

            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'path_topic': '/planned_path_smoother',
            'params_topic': '/smoother_params',

            # productban ez is kell, hogy tudja hol van a best_latest.pth
            'runs_dir': './ppo_runs',
            'model_path': './ppo_runs/best_latest.pth',
        }]
    )



    # 5) Nav2 bringup (benne van a Nav2PathClient + controller_server stb.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch)
    )

    # 6) TF broadcaster (odom -> base_link)
    tf_broadcaster = Node(
        package='hibrid_ai_pkg',
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

    # 8) Ignition odom -> ROS /odom
    gz_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=['/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/model/vehicle_blue/odometry', '/odom')],
        output='screen'
    )

    # 9) Ignition lidar -> ROS /scan
    gz_bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/lidar', '/scan')],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    metrics_logger = Node(
        package='metrics_pkg',
        executable='metrics_log',
        name='metrics_log',
        output='screen',
        parameters=[{
            'modszer': 'hibrid_d_star_lite_ppo_nav2_static',      
            'palya': LaunchConfiguration('map_file'), 
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel',
            'scan_topic': '/scan',
            'path_topic': '/planned_path_dilated',  
            'csv_dir': './metrics_runs',
            'need_goal_to_finish': False, 
            'stop_speed_eps': 0.05,
            'stop_time_s': 1.5
        }]
    )


    return LaunchDescription([
        map_file_arg,
        map_publication,
        d_star_lite_path_planner,
        trajectory_smoother,
        ppo_product,
        #rviz,
        nav2_bringup,
        gz_cmd_vel_bridge,
        gz_bridge_odom,
        tf_broadcaster,
        gz_bridge_lidar,
        metrics_logger
    ])
