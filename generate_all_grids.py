from tfg_robot.sdf_to_grid import create_grid_from_world
import numpy as np
import time

maps = {
    "simple": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world",
    "medium": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_medium.world",
    "complex": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_complex.world"
}

for name, path in maps.items():

    grid = create_grid_from_world(
        world_file=path,
        resolution=0.2,
        world_size=30.0
    )

    save_path = f"/home/yilun/tfg_ws/maps/{name}_grid.npy"

    np.save(save_path, grid)

    print(f"Saved: {save_path}")
