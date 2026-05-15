from tfg_robot.sdf_to_grid import create_grid_from_world
from tfg_robot.dijkstra_planner import dijkstra

import matplotlib.pyplot as plt
import math
import os
import time


maps = [
    ("simple", "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_simple.world"),
    ("medium", "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_medium.world"),
    ("complex", "/home/yilun/tfg_ws/src/tfg_worlds/worlds/experimento_complex.world")
]


def compute_path_length(path):
    total = 0

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        total += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    return total


os.makedirs("results_dijkstra", exist_ok=True)


for map_name, world_file in maps:

    print("\n==============================")
    print("Running Dijkstra on:", map_name)

    grid = create_grid_from_world(
        world_file=world_file,
        resolution=0.2,
        world_size=25.0
    )

    start = (10, 10)
    goal = (115, 115)

    start_time = time.perf_counter()
    path, visited_nodes = dijkstra(grid, start, goal)
    end_time = time.perf_counter()

    execution_time = end_time - start_time

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, origin="lower", cmap="gray_r")

    if path is not None:

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        path_length = compute_path_length(path)

        print("Visited nodes:", visited_nodes)
        print("Path length:", round(path_length, 2))
        print("Execution time:", round(execution_time, 6), "seconds")

        plt.plot(xs, ys, linewidth=2)
        plt.scatter(start[0], start[1], marker="o")
        plt.scatter(goal[0], goal[1], marker="x")

        plt.title(f"Dijkstra - {map_name}")

    else:
        print("No path found")
        print("Execution time:", round(execution_time, 6), "seconds")
        plt.title(f"No path - {map_name}")

    plt.xlabel("X")
    plt.ylabel("Y")

    save_path = f"results_dijkstra/{map_name}.png"

    plt.savefig(save_path)
    print("Saved:", save_path)

    plt.close()


print("\nAll Dijkstra experiments finished.")