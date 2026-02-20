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

    map_file_arg = DeclareLaunchArgument('map_file', default_value='occupancy_grid_3.csv')
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
        parameters=[map_publication_parameter_file, {'map_file': full_map_path}, {'use_sim_time': True}]
    )

    # 2) D* Lite -> /planned_path_dilated
    d_star_lite_path_planner = Node(
        package='hibrid_ai_pkg',
        executable='d_star_lite_path_planner',
        name='d_star_lite_path_planner',
        output='screen',
        parameters=[path_planner_parameter_file, {'map_file': full_map_path}, {'scenario': 'static'}, {'use_sim_time': True}],
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
            'params_topic': '/smoother_params',
            'use_sim_time': True
        }]
    )

    # 4) PPO trainer
    ppo_trainer = Node(
        package='hibrid_ai_pkg',
        executable='ppo_trainer',
        name='ppo_trainer',
        output='screen',
        parameters=[{
            'train_mode': True,
            'control_hz': 10.0,
            'max_steps': 2200, #1400, 1800

            'goal_tolerance': 0.7, #0.8, 0,5, 0,6
            'collision_distance': 0.18,

            'lidar_bins': 12,
            'lidar_max_range': 6.0,
            'max_offset_m': 0.10,

            # óvatos eltolás limit
            'offset_limit': 0.05,
            
            'smooth_max': 0.30, #0.25, 0.20 (ezzel rosszabb lett a lépés), 0,28 (ezzel rosszabb lett a lépés)

            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'path_topic': '/planned_path_smoother',
            'params_topic': '/smoother_params',

            # hol hozza létre az új run mappát
            'runs_dir': './ppo_runs/grid_3',
            'min_steps_for_goal': 50,
            
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
        parameters=[{'odom_topic': '/odom'},{'use_sim_time': True}]
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
    
    gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        map_file_arg,
        map_publication,
        d_star_lite_path_planner,
        trajectory_smoother,
        ppo_trainer,
        #rviz,
        nav2_bringup,
        gz_cmd_vel_bridge,
        gz_bridge_odom,
        gz_clock_bridge,
        tf_broadcaster,
        gz_bridge_lidar,
    ])
