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

 
    map_publication = Node(
        package='hibrid_ai_pkg',
        executable='map_publication',
        name='map_publication',
        output='screen',
        parameters=[map_publication_parameter_file, {'map_file': full_map_path}, {'use_sim_time': True}]
    )

  
    d_star_lite_path_planner = Node(
        package='hibrid_ai_pkg',
        executable='d_star_lite_path_planner',
        name='d_star_lite_path_planner',
        output='screen',
        parameters=[path_planner_parameter_file, {'map_file': full_map_path}, {'scenario': 'static'}, {'use_sim_time': True}],
    )

   
    ppo_trainer = Node(
        package='hibrid_ai_pkg',
        executable='ppo_trainer',
        name='ppo_trainer',
        output='screen',
        parameters=[{
            'train_mode': True,
            'control_hz': 10.0,
            'max_steps': 2600, #1400, 1800

            'goal_tolerance': 0.6, #0.8, 0.5, 0.6, 0.7
            'collision_distance': 0.18,

            'lidar_bins': 12,
            'lidar_max_range': 6.0,

            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'path_topic': '/planned_path_dilated',

            # hol hozza létre az új run mappát
            'runs_dir': './ppo_runs/grid_1',
            'min_steps_for_goal': 50,
            
            'cmd_vel_topic': '/cmd_vel',
            'energy_weight': 0.005,
            'stuck_window_steps': 80,
            'stuck_delta_eps': 0.001
                        
        }]
    )


    nav2_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_bringup_launch))

    tf_broadcaster = Node(
        package='hibrid_ai_pkg',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        parameters=[{'odom_topic': '/odom'}, {'use_sim_time': True}]
    )

 
    gz_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_cmd_vel_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist']
    )

  
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
        ppo_trainer,
        #rviz,
        nav2_bringup,
        gz_cmd_vel_bridge,
        gz_bridge_odom,
        gz_clock_bridge,
        tf_broadcaster,
        gz_bridge_lidar,
    ])
