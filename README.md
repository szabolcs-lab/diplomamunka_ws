# Mesterséges intelligenciával támogatot útvonaltervezés ROS2-ben

## Követelmények:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Ignition Gazebo Fortress
- git
- colcon
- rosdep

## A repository letöltése:

```bash 
git clone -b ppo https://github.com/szabolcs-lab/diplomamunka_ws.git
```

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
cd ~/diplomamunka_ws    
rosdep install --from-paths src --ignore-src -r -y
```

## A workspace buildelése:

```bash 
cd diplomamunka_ws
colcon build --symlink-install
source install/setup.bash
```

## Pályák indítása (Ignition Gazebo):

- A pályák elindítása külön terminálban történik.

```bash    
cd ~/diplomamunka_ws/src/simulation_resources_pkg/worlds
ign gazebo -r custom_world_1.sdf
```
vagy

```bash    
cd ~/diplomamunka_ws/src/simulation_resources_pkg/worlds
ign gazebo -r custom_world_2.sdf
```
vagy

```bash    
cd ~/diplomamunka_ws/src/simulation_resources_pkg/worlds
ign gazebo -r custom_world_3.sdf
```

## Klasszikus algoritmusok indítása:

- Egy másik terminálban:

```bash
cd ~/diplomamunka_ws
source install/setup.bash
```

- package lehet: a_star_pkg, d_star_lite_pkg, rrt_star_pkg; 
- map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv;

### Statikus pályák futtatása: 

```bash 
ros2 launch a_star_pkg static_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch a_star_pkg static_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch a_star_pkg static_launch.py map_file:=occupancy_grid_3.csv
```

vagy

```bash 
ros2 launch d_star_lite_pkg static_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch d_star_lite_pkg static_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch d_star_lite_pkg static_launch.py map_file:=occupancy_grid_3.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg static_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg static_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg static_launch.py map_file:=occupancy_grid_3.csv
```

### Dinamikus pályák futtatása: 

```bash 
ros2 launch a_star_pkg dynamic_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch a_star_pkg dynamic_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch a_star_pkg dynamic_launch.py map_file:=occupancy_grid_3.csv
```
vagy

```bash 
ros2 launch d_star_lite_pkg dynamic_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch d_star_lite_pkg dynamic_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch d_star_lite_pkg dynamic_launch.py map_file:=occupancy_grid_3.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg dynamic_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg dynamic_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch rrt_star_pkg dynamic_launch.py map_file:=occupancy_grid_3.csv
```

## Hibrid-D*Lite-NAV2-PPO algoritmus edzés:

- Egy másik terminálban:

```bash
cd ~/diplomamunka_ws
source install/setup.bash
```

- map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv;

```bash 
ros2 launch hibrid_ai_pkg trainer_grid_1_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg trainer_grid_2_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg trainer_grid_3_launch.py map_file:=occupancy_grid_3.csv
```

## Hibrid-D*Lite-NAV2-PPO algoritmus futattás - statikus:

- Egy másik terminálban:

```bash
cd ~/diplomamunka_ws
source install/setup.bash
```

- map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv;

```bash 
ros2 launch hibrid_ai_pkg static_grid_1_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg static_grid_2_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg static_grid_3_launch.py map_file:=occupancy_grid_3.csv
```

## Hibrid-D*Lite-NAV2-PPO algoritmus futattás - dinamikus:

- Egy másik terminálban:

```bash
cd ~/diplomamunka_ws
source install/setup.bash
```

- map_file lehet: occupancy_grid_1.csv, occupancy_grid_2.csv, occupancy_grid_3.csv;

```bash 
ros2 launch hibrid_ai_pkg dynamic_grid_1_launch.py map_file:=occupancy_grid_1.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg dynamic_grid_2_launch.py map_file:=occupancy_grid_2.csv
```
vagy

```bash 
ros2 launch hibrid_ai_pkg dynamic_grid_3_launch.py map_file:=occupancy_grid_3.csv
```



