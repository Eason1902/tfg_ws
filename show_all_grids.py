from tfg_robot.sdf_to_grid import create_grid_from_world
import matplotlib.pyplot as plt

maps = {
    "Simple": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world",
    "Medium": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_medium.world",
    "Complex": "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_complex.world"
}

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

for ax, (title, path) in zip(axes, maps.items()):

    grid = create_grid_from_world(
        world_file=path,
        resolution=0.2,
        world_size=30.0
    )

    ax.imshow(grid.T[:, ::-1], origin="lower", cmap="gray_r")
    ax.set_title(f"{title} Occupancy Grid")
    ax.set_xlabel("X Grid")
    ax.set_ylabel("Y Grid")

plt.tight_layout()
plt.show()