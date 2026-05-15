from tfg_robot.sdf_to_grid import create_grid_from_world
from tfg_robot.astar_planner import astar
from tfg_robot.path_utils import path_to_world

world_file = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world"

grid = create_grid_from_world(
    world_file=world_file,
    resolution=0.2,
    world_size=30.0
)

start = (20, 20)
goal = (130, 130)

path, visited_nodes = astar(grid, start, goal)

world_path = path_to_world(
    path,
    world_size=30.0,
    resolution=0.2
)

print("Grid path first points:")
print(path[:5])

print("World path first points:")
print(world_path[:5])

print("Total waypoints:", len(world_path))
