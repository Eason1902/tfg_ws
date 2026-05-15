from tfg_robot.sdf_to_grid import create_grid_from_world
import matplotlib.pyplot as plt

world_file = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world"

grid = create_grid_from_world(
    world_file=world_file,
    resolution=0.2,
    world_size=20.0
)

plt.imshow(grid, origin="lower", cmap="gray_r")
plt.title("Occupancy Grid from Gazebo World")
plt.xlabel("X grid")
plt.ylabel("Y grid")
plt.show()
