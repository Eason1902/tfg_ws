import heapq
import math


def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


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


def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    visited_nodes = 0

    while open_set:
        current = heapq.heappop(open_set)[1]
        visited_nodes += 1

        if current == goal:
            path = reconstruct_path(came_from, current)
            return path, visited_nodes

        for neighbor, move_cost in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + move_cost

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, visited_nodes