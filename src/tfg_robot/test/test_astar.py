from tfg_robot.sdf_to_grid import create_grid_from_world
from tfg_robot.astar_planner import astar
import matplotlib.pyplot as plt


world_file = "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world"

grid = create_grid_from_world(
    world_file=world_file,
    resolution=0.2,
    world_size=20.0
)

start = (10, 10)
goal = (80, 80)

path, visited_nodes = astar(grid, start, goal)

print("Visited nodes:", visited_nodes)

plt.imshow(grid, origin="lower", cmap="gray_r")

if path is not None:
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    plt.plot(xs, ys, linewidth=2)
    plt.scatter(start[0], start[1], marker="o")
    plt.scatter(goal[0], goal[1], marker="x")
    plt.title("A* Path Planning - Simple Map")
else:
    plt.title("No path found")

plt.xlabel("X grid")
plt.ylabel("Y grid")
plt.savefig("astar_simple.png")
plt.show()
