# Mesterséges intelligenciával támogatot útvonaltervezés ROS2-ben

## Követelmények:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Ignition Gazebo Fortress
- git
- colcon
- rosdep

## A szükséges ROS2 függőségek:

- rclpy
- nav_msgs
- std_msgs
- geometry_msgs
- ament_index_python
- rviz2
- nav2_msgs
- tf2_ros
- nav2_controller
- nav2_lifecycle_manager

## A függőségek telepítése:

```bash    
rosdep install --from-paths src --ignore-src -r -y
```

## A repository letöltése:

```bash 
git clone -b dynamic-obstacle https://github.com/szabolcs-lab/diplomamunka_ws.git
```

## A workspace buildelése:

```bash 
cd diplomamunka_ws
colcon build --symlink-install
source install/setup.bash
```

## Pályák indítása (Ignition Gazebo):

- A pályák elindítása külön terminálban történik. A custom_world lehet: custom_world_1.sdf, custom_world_2.sdf, custom_world_3.sdf.

```bash    
cd ~/diplomamunka_ws/src/simulation_resources_pkg/worlds
ign gazebo -r custom_world_1.sdf
```

## Algoritmusok indítása:

Egy másik terminálban:

```bash
cd ~/diplomamunka_ws
source install/setup.bash
```

Statikus pálya futtatása (package lehet: a_star_pkg, d_star_lite_pkg, rrt_star_pkg; map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv):

```bash 
ros2 launch d_star_lite_pkg static_launch.py map_file:=occupancy_grid_1.csv
```
Dinamikus pálya futtatása (package lehet: a_star_pkg, d_star_lite_pkg, rrt_star_pkg; map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv):

```bash 
ros2 launch d_star_lite_pkg dynamic_launch.py map_file:=occupancy_grid_1.csv
```
