import numpy as np

def generate_occupancy_grid(walls, grid_size_x, grid_size_y, resolution):
    """
    walls: list of dict, mindegyik: {'x', 'y', 'size_x', 'size_y', 'yaw'}
    grid_size_x, grid_size_y: világ mérete méterben
    resolution: méter / cella
    visszatér egy 2D numpy int8 mátrixszal (0 szabad, 1 akadály)
    """
    cells_x = int(grid_size_x / resolution)
    cells_y = int(grid_size_y / resolution)
    grid = np.zeros((cells_y, cells_x), dtype=np.int8)

    def world_to_grid(x, y):
        gx = int((x + grid_size_x/2) / resolution)
        gy = int((y + grid_size_y/2) / resolution)
        return gx, gy

    from math import cos, sin

    for wall in walls:
        x = wall['x']
        y = wall['y']
        sx = wall['size_x']
        sy = wall['size_y']
        yaw = wall.get('yaw', 0)

        # Először vegyük az akadály téglalapját a világ koordinátáiban (szög nélkül)
        # Majd cellák szinten ellenőrizzük, hogy az adott cella a forgatott téglalapon belül van-e

        # Grid cellák koordinátái
        for gy in range(cells_y):
            for gx in range(cells_x):
                # Világ koordináta az adott cella közepére
                wx = gx * resolution - grid_size_x/2 + resolution/2
                wy = gy * resolution - grid_size_y/2 + resolution/2

                # Transzformáció: cella koordináta eltolása az akadály középpontjához
                dx = wx - x
                dy = wy - y

                # Forgatás ellentétes irányba (hogy az akadály tengelyeihez igazítsuk)
                dx_rot = dx * cos(-yaw) - dy * sin(-yaw)
                dy_rot = dx * sin(-yaw) + dy * cos(-yaw)

                # Ha az adott pont az akadály téglalapján belül van
                if (-sx/2 <= dx_rot <= sx/2) and (-sy/2 <= dy_rot <= sy/2):
                    grid[gy, gx] = 1

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
                <uri>file://./my_robot_3.sdf</uri>
                <pose>-10 10 0 0 0 0</pose>
            </include>
        """

    sdf_footer = '''
    </world>
</sdf>
'''
    return sdf_header + sdf_walls + robot_model + sdf_footer


def main():
    # Beállítások
    grid_size_x = 20  # méter
    grid_size_y = 20
    resolution = 0.1  # méter / cella

    # Akadályok definiálása - ezt tetszőlegesen bővítheted, átírhatod
    walls = [
        {"name": "wall1", "x": 5, "y": 0, "size_x": 0.5, "size_y": 10, "yaw": 0},
        {"name": "wall2", "x": 0, "y": 2, "size_x": 4, "size_y": 0.5, "yaw": 0},
        {"name": "wall3", "x": 8, "y": 2, "size_x": 1, "size_y": 10, "yaw": 0},
        {"name": "wall4", "x": 1, "y": -5, "size_x": 0.5, "size_y": 10, "yaw": 0},
        {"name": "wall5", "x": -4, "y": 2.5, "size_x": 0.5, "size_y": 15, "yaw": 0},
        {"name": "wall6", "x": -8, "y": 3, "size_x": 4, "size_y": 0.5, "yaw": 0},
        {"name": "box1", "x": -2, "y": -3, "size_x": 1, "size_y": 1, "yaw": 0},
        {"name": "box2", "x": -5, "y": -3, "size_x": 2.5, "size_y": 1, "yaw": 0},
        #{"name": "box3", "x": -10, "y": 10, "size_x": 1, "size_y": 1, "yaw": 0} # majd a robot kezdőpozija
    ]

    # Occupancy grid generálása
    grid = generate_occupancy_grid(walls, grid_size_x, grid_size_y, resolution)

    # Mentés CSV-be
    np.savetxt("occupancy_grid_1.csv", grid, fmt="%d", delimiter=",")

    # SDF generálása
    sdf_text = generate_sdf(walls, grid_size_x, grid_size_y)
    with open("custom_world_1.sdf", "w") as f:
        f.write(sdf_text)

    print("Generált occupancy_grid_1.csv és custom_world_1.sdf fájlokat.")

if __name__ == "__main__":
    main()
