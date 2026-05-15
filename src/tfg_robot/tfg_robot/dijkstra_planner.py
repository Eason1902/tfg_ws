import heapq
import math


def get_neighbors(node, grid):
    x, y = node
    neighbors = []

    movements = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    height = grid.shape[0]
    width = grid.shape[1]

    for dx, dy in movements:
        nx = x + dx
        ny = y + dy

        if 0 <= nx < width and 0 <= ny < height:
            if grid[ny][nx] == 0:
                cost = math.sqrt(2) if dx != 0 and dy != 0 else 1
                neighbors.append(((nx, ny), cost))

    return neighbors


def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def dijkstra(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    cost_so_far = {start: 0}

    visited_nodes = 0

    while open_set:
        current_cost, current = heapq.heappop(open_set)
        visited_nodes += 1

        if current == goal:
            path = reconstruct_path(came_from, current)
            return path, visited_nodes

        for neighbor, move_cost in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + move_cost

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(open_set, (new_cost, neighbor))

    return None, visited_nodes
