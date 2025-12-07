import numpy as np
from math import cos, sin
import os
from ament_index_python.packages import get_package_share_directory

def generate_occupancy_grid(walls, grid_size_x, grid_size_y, resolution):
    """
    walls: dictionary lista, mindegyik: {'x', 'y', 'size_x', 'size_y', 'yaw'} paraméterekkel
    grid_size_x, grid_size_y: világ mérete méterben
    resolution: méter / cella
    visszatérés egy 2D numpy int8 mátrixszal (0 szabad, 1 akadály)
    """
    
    # a vilg koordinátákat itt alakítjuk át gridre, ahol a sor az y lesz és az x az oszlop lesz 
    cells_x = int(grid_size_x / resolution)
    cells_y = int(grid_size_y / resolution)
    grid = np.zeros((cells_y, cells_x), dtype=np.int8) # 2D numpy tömb, ami a rács lesz

    def world_to_grid(x, y):
        grid_x = int((x + grid_size_x/2) / resolution)
        grid_y = int((y + grid_size_y/2) / resolution)
        return grid_x, grid_y
 
    # a falak listáján végigmegyünk
    for wall in walls:
        x = wall['x']
        y = wall['y']
        size_x = wall['size_x']
        size_y = wall['size_y']
        yaw = wall.get('yaw', 0)

        # Grid cellák koordinátái
        for grid_y in range(cells_y):
            for grid_x in range(cells_x):
                
                # itt számoljuk ki, hogy a grid_x és grid_y indexekből milyen abszolút koordináta lesz a világban, 
                # figyelembe véve a rács cellaméretét (resolution) és a rács teljes méretét
                world_x = grid_x * resolution - grid_size_x/2 + resolution/2
                world_y = grid_y * resolution - grid_size_y/2 + resolution/2

                # itt kiszámoljuk, hogy az adott cella mennyire van eltolva az akadály közepétől a világ koordinátáiban
                direction_x = world_x - x
                direction_y = world_y - y

                # itt forgatjuk el a cella koordinátáit az akadály tengelyeihez képest
                x_relative_to_obstacle = direction_x * cos(-yaw) - direction_y * sin(-yaw)
                y_relative_to_obstacle = direction_x * sin(-yaw) + direction_y * cos(-yaw)

                # ha az adott pont az akadály téglalapján belül van
                if (-size_x/2 <= x_relative_to_obstacle <= size_x/2) and (-size_y/2 <= y_relative_to_obstacle <= size_y/2):
                    grid[grid_y, grid_x] = 1

    return grid

def generate_sdf(walls, grid_size_x, grid_size_y):
    """
    Létrehoz egy sdf szöveget a megadott akadályokkal, és egy alap világmodelllel.
    """
    sdf_header = '''<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="custom_world">
        <physics name="default_physics" type="ignition">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="libignition-gazebo-imu-system.so"
            name="ignition::gazebo::systems::Imu">
        </plugin>
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>{gs_x} {gs_y}</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.8 1</ambient>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                </visual>
            </link>
        </model>
'''.format(gs_x=grid_size_x, gs_y=grid_size_y)

    sdf_walls = ""
    for i, wall in enumerate(walls):
        name = wall['name'] if 'name' in wall else f"wall_{i}"
        x = wall['x']
        y = wall['y']
        z = wall.get('z', 1)  # alapból 1m magas akadály
        yaw = wall.get('yaw', 0)
        size_x = wall['size_x']
        size_y = wall['size_y']
        size_z = wall.get('size_z', 2)

        # SDF-ben a pozíció 6 érték: x y z roll pitch yaw
        sdf_walls += f'''
        <model name="{name}">
            <static>true</static>
            <pose>{x} {y} {z} 0 0 {yaw}</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>{size_x} {size_y} {size_z}</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>{size_x} {size_y} {size_z}</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.7 0.2 0.2 1</ambient>
                        <diffuse>0.7 0.2 0.2 1</diffuse>
                        <specular>0.7 0.2 0.2 1</specular>
                    </material>
                </visual>
            </link>
        </model>
'''
    robot_model = f"""
            <include>
                <uri>file://../model/my_robot_3.sdf</uri>
                <pose>-10 10 0 0 0 0</pose>
            </include>
        """

    sdf_footer = '''
    </world>
</sdf>
'''
    return sdf_header + sdf_walls + robot_model + sdf_footer


def main():
    # beállítások 
    grid_size_x = 20  # méter
    grid_size_y = 20
    resolution = 0.1  # méter / cella

    # akadályok definiálása
    walls = [
        {"name": "wall1", "x": 5, "y": 0, "size_x": 0.5, "size_y": 10, "yaw": 0},
        {"name": "wall2", "x": 0, "y": 2, "size_x": 4, "size_y": 0.5, "yaw": 0},
        {"name": "wall3", "x": 8, "y": 2, "size_x": 1, "size_y": 10, "yaw": 0},
        {"name": "wall4", "x": 1, "y": -5, "size_x": 0.5, "size_y": 10, "yaw": 0},
        {"name": "wall5", "x": -4, "y": 2.5, "size_x": 0.5, "size_y": 15, "yaw": 0},
        {"name": "wall6", "x": -8, "y": 3, "size_x": 4, "size_y": 0.5, "yaw": 0},
        {"name": "box1", "x": -2, "y": -3, "size_x": 1, "size_y": 1, "yaw": 0},
        {"name": "box2", "x": -5, "y": -3, "size_x": 2.5, "size_y": 1, "yaw": 0},
    ]

    # OccupancyGrid generálása
    grid = generate_occupancy_grid(walls, grid_size_x, grid_size_y, resolution)
    
    # jelenlegi fájl abszolút elérési útja, pontosabban az a mappa '..', ahol van
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    
    # hozzáfűzzük a maps és worlds mappákat, ezek lesznek a fájlok helyei
    dir_csv = os.path.join(base_dir, "maps")
    dir_sdf = os.path.join(base_dir, "worlds")
    
    # ha a mappák nem léteznek, akkor létrehozzuk
    os.makedirs(dir_csv, exist_ok=True)
    os.makedirs(dir_sdf, exist_ok=True)
    
    # fájlok neveu
    name_csv = "occupancy_grid_1.csv"
    name_sdf = "custom_world_1.sdf"
    
    # ahová létrejönnek a fájlok
    path_csv = os.path.join(dir_csv, name_csv)
    path_sdf = os.path.join(dir_sdf, name_sdf)

    # mentés CSV-be
    np.savetxt(path_csv, grid, fmt="%d", delimiter=",")

    # SDF generálása
    sdf_text = generate_sdf(walls, grid_size_x, grid_size_y)
    with open(path_sdf, "w") as f:
        f.write(sdf_text)

    print("Generált occupancy_grid_1.csv és custom_world_1.sdf fájlokat.")

if __name__ == "__main__":
    main()
